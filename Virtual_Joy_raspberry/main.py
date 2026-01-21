#!/usr/bin/env python3
"""
Virtual Joystick for VR2 Wheelchair using Head Tracking
Main entry point

Controls wheelchair movement by tracking head position:
- Turn head left/right -> Steer left/right
- Tilt head up/down -> Move forward/backward

Keys:
- 'c': Calibrate neutral head position
- 'r': Reset calibration
- 'b': Connect/reconnect BLE
- 's': Send START command
- 'x': Send SHUTDOWN command
- '+': Speed up
- '-': Speed down
- 'p': Toggle pause (disable joystick output)
- 'q': Quit
"""

import asyncio
import time
import signal
import sys
import argparse
from typing import Optional

import cv2

import config
from head_tracker import HeadTracker, HeadPose
from joy_mapper import JoyMapper, JoystickState, draw_joystick_overlay
from ble_client import BLEClient, Telemetry, WheelchairState


class VirtualJoystick:
    """
    Main application class combining head tracking with BLE joystick control.
    """

    def __init__(self, use_ble: bool = True, use_display: bool = True):
        """
        Initialize virtual joystick.

        Args:
            use_ble: Enable BLE communication
            use_display: Show debug display window
        """
        self.use_ble = use_ble
        self.use_display = use_display

        # Components
        self.tracker = HeadTracker()
        self.mapper = JoyMapper()
        self.ble_client: Optional[BLEClient] = None

        # Camera
        self.cap = None
        self.camera_initialized = False

        # State
        self.running = False
        self.paused = False
        self.last_ble_send = 0.0
        self.ble_interval = 1.0 / config.BLE_UPDATE_RATE

        # FPS tracking
        self.fps = 0.0
        self.frame_count = 0
        self.fps_start_time = 0.0

        # Telemetry
        self.last_telemetry: Optional[Telemetry] = None

    def _init_camera(self) -> bool:
        """Initialize camera (Raspberry Pi Camera 3 or USB webcam)"""
        if config.USE_LIBCAMERA:
            # Try picamera2 first (Raspberry Pi Camera 3)
            try:
                from picamera2 import Picamera2

                self.picam2 = Picamera2()

                # Configure camera
                cam_config = self.picam2.create_preview_configuration(
                    main={"size": (config.CAMERA_WIDTH, config.CAMERA_HEIGHT),
                          "format": "RGB888"}
                )
                self.picam2.configure(cam_config)
                self.picam2.start()

                if config.DEBUG_CONSOLE:
                    print(f"[CAMERA] Initialized Raspberry Pi Camera 3 "
                          f"({config.CAMERA_WIDTH}x{config.CAMERA_HEIGHT})")

                self.camera_initialized = True
                self.use_picamera2 = True
                return True

            except ImportError:
                if config.DEBUG_CONSOLE:
                    print("[CAMERA] picamera2 not available, trying OpenCV...")
            except Exception as e:
                if config.DEBUG_CONSOLE:
                    print(f"[CAMERA] picamera2 error: {e}, trying OpenCV...")

        # Fall back to OpenCV (USB webcam)
        try:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)

            if self.cap.isOpened():
                if config.DEBUG_CONSOLE:
                    actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    print(f"[CAMERA] Initialized USB webcam ({actual_w}x{actual_h})")

                self.camera_initialized = True
                self.use_picamera2 = False
                return True

        except Exception as e:
            if config.DEBUG_CONSOLE:
                print(f"[CAMERA] OpenCV error: {e}")

        print("[ERROR] No camera available!")
        return False

    def _get_frame(self):
        """Get frame from camera"""
        if not self.camera_initialized:
            return None

        if self.use_picamera2:
            # Raspberry Pi Camera
            frame = self.picam2.capture_array()
            # Convert RGB to BGR for OpenCV
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        else:
            # USB webcam
            ret, frame = self.cap.read()
            if ret:
                return frame
            return None

    def _on_telemetry(self, telemetry: Telemetry):
        """Callback for BLE telemetry updates"""
        self.last_telemetry = telemetry
        if config.DEBUG_CONSOLE:
            state_name = telemetry.state.name
            print(f"[TELE] State: {state_name}, Speed: {telemetry.speed_level}")

    async def _init_ble(self) -> bool:
        """Initialize BLE connection"""
        if not self.use_ble:
            return True

        self.ble_client = BLEClient(on_telemetry=self._on_telemetry)

        # Scan for device
        address = await self.ble_client.scan(timeout=10)
        if not address:
            print("[BLE] Device not found. Running in offline mode.")
            return False

        # Connect
        if not await self.ble_client.connect(address):
            print("[BLE] Failed to connect. Running in offline mode.")
            return False

        print("[BLE] Connected successfully!")
        return True

    async def _send_joystick_ble(self, state: JoystickState):
        """Send joystick state via BLE"""
        if not self.ble_client or not self.ble_client.connected:
            return

        if self.paused:
            # Send neutral when paused
            await self.ble_client.send_joystick(0, 0, 0)
        else:
            await self.ble_client.send_joystick(state.x, state.y, state.buttons)

    def _update_fps(self):
        """Update FPS counter"""
        self.frame_count += 1
        elapsed = time.time() - self.fps_start_time

        if elapsed >= config.FPS_UPDATE_INTERVAL:
            self.fps = self.frame_count / elapsed
            self.frame_count = 0
            self.fps_start_time = time.time()

    def _draw_status_overlay(self, frame, pose: HeadPose, joy_state: JoystickState):
        """Draw status information overlay"""
        height = frame.shape[0]

        # BLE status
        if self.use_ble and self.ble_client:
            if self.ble_client.connected:
                ble_status = "BLE: CONNECTED"
                ble_color = (0, 255, 0)
            else:
                ble_status = "BLE: DISCONNECTED"
                ble_color = (0, 0, 255)
        else:
            ble_status = "BLE: DISABLED"
            ble_color = (128, 128, 128)

        cv2.putText(frame, ble_status, (10, height - 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, ble_color, 1)

        # Telemetry info
        if self.last_telemetry:
            tele_text = f"ECU: {self.last_telemetry.state.name} | Speed: {self.last_telemetry.speed_level}"
            cv2.putText(frame, tele_text, (10, height - 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Pause status
        if self.paused:
            cv2.putText(frame, "** PAUSED **", (10, height - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # FPS
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Instructions
        instructions = "c:Calibrate r:Reset b:BLE s:Start x:Stop +/-:Speed p:Pause q:Quit"
        cv2.putText(frame, instructions, (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

        return frame

    async def _handle_key(self, key: int) -> bool:
        """
        Handle keyboard input.
        Returns False if should quit.
        """
        if key == ord('q'):
            return False

        elif key == ord('c'):
            # Calibrate
            frame = self._get_frame()
            if frame is not None:
                if self.tracker.calibrate(frame):
                    print("[INFO] Calibration successful!")
                else:
                    print("[INFO] Calibration failed - no face detected")

        elif key == ord('r'):
            # Reset calibration
            self.tracker.reset_calibration()
            print("[INFO] Calibration reset")

        elif key == ord('p'):
            # Toggle pause
            self.paused = not self.paused
            status = "PAUSED" if self.paused else "ACTIVE"
            print(f"[INFO] Joystick output {status}")

        elif key == ord('b'):
            # Reconnect BLE
            if self.use_ble:
                if self.ble_client:
                    await self.ble_client.disconnect()
                print("[INFO] Reconnecting BLE...")
                await self._init_ble()

        elif key == ord('s'):
            # Send START
            if self.ble_client and self.ble_client.connected:
                await self.ble_client.send_start()
                print("[INFO] Sent START command")

        elif key == ord('x'):
            # Send SHUTDOWN
            if self.ble_client and self.ble_client.connected:
                await self.ble_client.send_shutdown()
                print("[INFO] Sent SHUTDOWN command")

        elif key == ord('+') or key == ord('='):
            # Speed up
            if self.ble_client and self.ble_client.connected:
                await self.ble_client.send_speed_plus()
                print("[INFO] Speed increased")

        elif key == ord('-'):
            # Speed down
            if self.ble_client and self.ble_client.connected:
                await self.ble_client.send_speed_minus()
                print("[INFO] Speed decreased")

        return True

    async def run(self):
        """Main run loop"""
        print("=" * 60)
        print("Virtual Joystick - Head Tracking Controller")
        print("=" * 60)

        # Initialize camera
        if not self._init_camera():
            return

        # Initialize BLE
        if self.use_ble:
            await self._init_ble()

        # Setup window
        if self.use_display:
            cv2.namedWindow("Virtual Joystick", cv2.WINDOW_NORMAL)

        print("\nRunning... Press 'c' to calibrate, 'q' to quit")
        print("-" * 60)

        self.running = True
        self.fps_start_time = time.time()

        try:
            while self.running:
                # Get frame
                frame = self._get_frame()
                if frame is None:
                    await asyncio.sleep(0.01)
                    continue

                # Process head tracking
                pose = self.tracker.process_frame(frame)

                # Map to joystick
                joy_state = self.mapper.map_pose_to_joy(pose)

                # Send via BLE at fixed rate
                current_time = time.time()
                if current_time - self.last_ble_send >= self.ble_interval:
                    await self._send_joystick_ble(joy_state)
                    self.last_ble_send = current_time

                # Update FPS
                self._update_fps()

                # Display
                if self.use_display:
                    # Draw debug overlay
                    display_frame = self.tracker.draw_debug(frame, pose)
                    display_frame = draw_joystick_overlay(display_frame, joy_state)
                    display_frame = self._draw_status_overlay(display_frame, pose, joy_state)

                    cv2.imshow("Virtual Joystick", display_frame)

                    # Handle keyboard
                    key = cv2.waitKey(1) & 0xFF
                    if key != 255:
                        if not await self._handle_key(key):
                            break

                # Small delay to prevent CPU overload
                await asyncio.sleep(0.001)

        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user")

        finally:
            await self.cleanup()

    async def cleanup(self):
        """Cleanup resources"""
        print("\nCleaning up...")

        self.running = False

        # Send stop command
        if self.ble_client and self.ble_client.connected:
            await self.ble_client.send_joystick(0, 0, 0)
            await asyncio.sleep(0.1)
            await self.ble_client.disconnect()

        # Close camera
        if self.camera_initialized:
            if self.use_picamera2:
                self.picam2.stop()
            elif self.cap:
                self.cap.release()

        # Close tracker
        self.tracker.close()

        # Close windows
        cv2.destroyAllWindows()

        print("Cleanup complete")


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description="Virtual Joystick - Control wheelchair with head movements"
    )
    parser.add_argument(
        "--no-ble",
        action="store_true",
        help="Disable BLE communication (test mode)"
    )
    parser.add_argument(
        "--no-display",
        action="store_true",
        help="Disable display window (headless mode)"
    )
    parser.add_argument(
        "--webcam",
        action="store_true",
        help="Force use of USB webcam instead of Pi Camera"
    )

    args = parser.parse_args()

    # Override config based on args
    if args.webcam:
        config.USE_LIBCAMERA = False

    # Create and run application
    app = VirtualJoystick(
        use_ble=not args.no_ble,
        use_display=not args.no_display
    )

    # Run async
    asyncio.run(app.run())


if __name__ == "__main__":
    main()
