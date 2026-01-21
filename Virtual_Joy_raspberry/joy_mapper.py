"""
Joystick Mapper Module
Maps head pose (yaw, pitch) to joystick coordinates (x, y)
"""

from dataclasses import dataclass
from typing import Tuple

import config
from head_tracker import HeadPose


@dataclass
class JoystickState:
    """Joystick state data structure"""
    x: int = 0       # -100 to +100 (left/right)
    y: int = 0       # -100 to +100 (backward/forward)
    buttons: int = 0  # Button flags
    active: bool = False  # True if joystick output is active


class JoyMapper:
    """
    Maps head pose angles to joystick coordinates.

    Mapping:
    - Yaw (left/right head rotation) -> X axis
    - Pitch (up/down head tilt) -> Y axis

    Features:
    - Dead zone filtering
    - Range limiting
    - Axis inversion
    - Safety limiting
    """

    def __init__(self):
        """Initialize the joy mapper"""
        # Range settings
        self.yaw_range = config.HEAD_YAW_RANGE
        self.pitch_range = config.HEAD_PITCH_RANGE

        # Dead zones
        self.dead_zone_yaw = config.DEAD_ZONE_YAW
        self.dead_zone_pitch = config.DEAD_ZONE_PITCH

        # Axis inversion
        self.invert_x = config.INVERT_X
        self.invert_y = config.INVERT_Y

        # Output limits
        self.joy_min = config.JOY_MIN
        self.joy_max = config.JOY_MAX

        # Safety limiting (percentage of max)
        self.max_output = int(config.JOY_MAX * config.MAX_OUTPUT_PERCENT / 100)

        # Current state
        self.current_state = JoystickState()

        # Button state (can be set externally)
        self.buttons = config.BLE_BTN_NONE

    def _apply_dead_zone(self, value: float, dead_zone: float) -> float:
        """Apply dead zone to a value"""
        if abs(value) < dead_zone:
            return 0.0

        # Scale the value to remove the dead zone gap
        sign = 1.0 if value > 0 else -1.0
        adjusted = abs(value) - dead_zone
        return sign * adjusted

    def _map_range(self, value: float, input_range: float, output_min: int, output_max: int) -> int:
        """Map a value from input range to output range"""
        # Normalize to -1 to 1
        normalized = value / input_range

        # Clamp to -1 to 1
        normalized = max(-1.0, min(1.0, normalized))

        # Map to output range
        output_range = output_max - output_min
        mapped = int(normalized * (output_range / 2))

        return mapped

    def map_pose_to_joy(self, pose: HeadPose) -> JoystickState:
        """
        Map head pose to joystick coordinates.

        Args:
            pose: HeadPose with yaw and pitch values

        Returns:
            JoystickState with x, y coordinates
        """
        # Check if tracking is active
        if not pose.face_detected and not config.JOYSTICK_ENABLED:
            self.current_state = JoystickState(
                x=0, y=0, buttons=self.buttons, active=False
            )
            return self.current_state

        # Get pose values
        yaw = pose.yaw
        pitch = pose.pitch

        # Apply dead zones
        yaw = self._apply_dead_zone(yaw, self.dead_zone_yaw)
        pitch = self._apply_dead_zone(pitch, self.dead_zone_pitch)

        # Map to joystick range
        # Yaw -> X: positive yaw (right) = positive X (right)
        x = self._map_range(yaw, self.yaw_range - self.dead_zone_yaw,
                           self.joy_min, self.joy_max)

        # Pitch -> Y: positive pitch (up) = positive Y (forward)
        y = self._map_range(pitch, self.pitch_range - self.dead_zone_pitch,
                           self.joy_min, self.joy_max)

        # Apply axis inversion
        if self.invert_x:
            x = -x
        if self.invert_y:
            y = -y

        # Apply safety limiting
        x = max(-self.max_output, min(self.max_output, x))
        y = max(-self.max_output, min(self.max_output, y))

        # Update state
        self.current_state = JoystickState(
            x=x,
            y=y,
            buttons=self.buttons,
            active=pose.face_detected and config.JOYSTICK_ENABLED
        )

        return self.current_state

    def set_button(self, button: int, pressed: bool):
        """Set a button state"""
        if pressed:
            self.buttons |= button
        else:
            self.buttons &= ~button

    def clear_buttons(self):
        """Clear all button states"""
        self.buttons = config.BLE_BTN_NONE

    def set_invert_x(self, invert: bool):
        """Set X axis inversion"""
        self.invert_x = invert

    def set_invert_y(self, invert: bool):
        """Set Y axis inversion"""
        self.invert_y = invert

    def set_sensitivity(self, yaw_range: float, pitch_range: float):
        """
        Set sensitivity by adjusting the input range.
        Smaller range = more sensitive.

        Args:
            yaw_range: Degrees of yaw for full X range
            pitch_range: Degrees of pitch for full Y range
        """
        self.yaw_range = max(5.0, yaw_range)  # Minimum 5 degrees
        self.pitch_range = max(5.0, pitch_range)

    def set_dead_zone(self, yaw_dead: float, pitch_dead: float):
        """Set dead zone values"""
        self.dead_zone_yaw = max(0.0, yaw_dead)
        self.dead_zone_pitch = max(0.0, pitch_dead)

    def set_max_output(self, percent: int):
        """Set maximum output as percentage of full range"""
        percent = max(10, min(100, percent))
        self.max_output = int(config.JOY_MAX * percent / 100)

    def get_debug_info(self) -> dict:
        """Get debug information"""
        return {
            'x': self.current_state.x,
            'y': self.current_state.y,
            'buttons': self.current_state.buttons,
            'active': self.current_state.active,
            'max_output': self.max_output,
            'yaw_range': self.yaw_range,
            'pitch_range': self.pitch_range,
            'dead_zone_yaw': self.dead_zone_yaw,
            'dead_zone_pitch': self.dead_zone_pitch,
            'invert_x': self.invert_x,
            'invert_y': self.invert_y
        }


def draw_joystick_overlay(frame, state: JoystickState, x_pos: int = None, y_pos: int = None):
    """
    Draw a joystick visualization overlay on a frame.

    Args:
        frame: OpenCV frame (BGR)
        state: JoystickState to visualize
        x_pos: X position for overlay (default: bottom right)
        y_pos: Y position for overlay (default: bottom right)
    """
    import cv2

    height, width = frame.shape[:2]

    # Overlay position (bottom right by default)
    overlay_size = 120
    margin = 20

    if x_pos is None:
        x_pos = width - overlay_size - margin
    if y_pos is None:
        y_pos = height - overlay_size - margin

    # Draw background circle
    center_x = x_pos + overlay_size // 2
    center_y = y_pos + overlay_size // 2
    radius = overlay_size // 2 - 5

    # Background
    cv2.circle(frame, (center_x, center_y), radius, (50, 50, 50), -1)
    cv2.circle(frame, (center_x, center_y), radius, (100, 100, 100), 2)

    # Draw crosshairs
    cv2.line(frame, (center_x - radius, center_y), (center_x + radius, center_y), (80, 80, 80), 1)
    cv2.line(frame, (center_x, center_y - radius), (center_x, center_y + radius), (80, 80, 80), 1)

    # Draw joystick position
    # Map -100..+100 to pixel offset
    scale = radius / 100.0
    joy_x = center_x + int(state.x * scale)
    joy_y = center_y - int(state.y * scale)  # Invert Y for display

    # Joystick indicator color based on active state
    if state.active:
        color = (0, 255, 0)  # Green when active
    else:
        color = (0, 0, 255)  # Red when inactive

    cv2.circle(frame, (joy_x, joy_y), 10, color, -1)
    cv2.circle(frame, (joy_x, joy_y), 10, (255, 255, 255), 2)

    # Draw line from center to position
    cv2.line(frame, (center_x, center_y), (joy_x, joy_y), color, 2)

    # Draw values text
    text_y = y_pos - 10
    cv2.putText(frame, f"X:{state.x:+4d} Y:{state.y:+4d}", (x_pos, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Draw status
    status = "ACTIVE" if state.active else "PAUSED"
    cv2.putText(frame, status, (x_pos, y_pos + overlay_size + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    return frame


# Test standalone
if __name__ == "__main__":
    import cv2
    import numpy as np

    print("Testing JoyMapper...")

    mapper = JoyMapper()

    # Create a test window
    test_frame = np.zeros((480, 640, 3), dtype=np.uint8)

    # Simulate different poses
    test_poses = [
        HeadPose(yaw=0, pitch=0, face_detected=True),      # Center
        HeadPose(yaw=15, pitch=0, face_detected=True),     # Right
        HeadPose(yaw=-15, pitch=0, face_detected=True),    # Left
        HeadPose(yaw=0, pitch=15, face_detected=True),     # Up/Forward
        HeadPose(yaw=0, pitch=-15, face_detected=True),    # Down/Backward
        HeadPose(yaw=20, pitch=15, face_detected=True),    # Right-Forward
        HeadPose(yaw=2, pitch=1, face_detected=True),      # In dead zone
        HeadPose(yaw=0, pitch=0, face_detected=False),     # No face
    ]

    print("\nTest mapping results:")
    print("-" * 50)

    for i, pose in enumerate(test_poses):
        state = mapper.map_pose_to_joy(pose)
        print(f"Pose {i+1}: yaw={pose.yaw:+6.1f}, pitch={pose.pitch:+6.1f}, "
              f"face={pose.face_detected} -> x={state.x:+4d}, y={state.y:+4d}, "
              f"active={state.active}")

        # Draw visualization
        frame = test_frame.copy()
        frame = draw_joystick_overlay(frame, state)

        cv2.putText(frame, f"Test {i+1}: yaw={pose.yaw:.1f}, pitch={pose.pitch:.1f}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        cv2.imshow("JoyMapper Test", frame)
        cv2.waitKey(500)

    print("-" * 50)
    print("\nDebug info:", mapper.get_debug_info())

    cv2.destroyAllWindows()
