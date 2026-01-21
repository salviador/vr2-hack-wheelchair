"""
Head Tracker Module using MediaPipe Face Mesh
Detects face and estimates head pose (yaw, pitch, roll)
"""

import cv2
import numpy as np
import mediapipe as mp
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import config


@dataclass
class HeadPose:
    """Head pose data structure"""
    yaw: float = 0.0      # Left/Right rotation (degrees)
    pitch: float = 0.0    # Up/Down rotation (degrees)
    roll: float = 0.0     # Tilt rotation (degrees)
    face_detected: bool = False
    timestamp: float = 0.0


class HeadTracker:
    """
    Head tracking using MediaPipe Face Mesh.
    Extracts head pose (yaw, pitch, roll) from facial landmarks.
    """

    # 3D model points for head pose estimation (standard face model)
    # These points correspond to specific facial landmarks
    MODEL_POINTS = np.array([
        (0.0, 0.0, 0.0),          # Nose tip (landmark 1)
        (0.0, -330.0, -65.0),     # Chin (landmark 152)
        (-225.0, 170.0, -135.0),  # Left eye corner (landmark 263)
        (225.0, 170.0, -135.0),   # Right eye corner (landmark 33)
        (-150.0, -150.0, -125.0), # Left mouth corner (landmark 287)
        (150.0, -150.0, -125.0)   # Right mouth corner (landmark 57)
    ], dtype=np.float64)

    # MediaPipe landmark indices for the 6 key points
    LANDMARK_INDICES = [1, 152, 263, 33, 287, 57]

    def __init__(self):
        """Initialize the head tracker"""
        # Initialize MediaPipe Face Mesh
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            static_image_mode=False,
            max_num_faces=config.MAX_NUM_FACES,
            min_detection_confidence=config.FACE_DETECTION_CONFIDENCE,
            min_tracking_confidence=config.FACE_TRACKING_CONFIDENCE,
            refine_landmarks=True
        )

        # Drawing utilities
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # Camera matrix (will be set based on frame size)
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float64)

        # Current pose
        self.current_pose = HeadPose()

        # Smoothing
        self.smoothed_yaw = 0.0
        self.smoothed_pitch = 0.0
        self.smoothed_roll = 0.0

        # Calibration
        self.neutral_yaw = config.NEUTRAL_YAW
        self.neutral_pitch = config.NEUTRAL_PITCH
        self.is_calibrated = False

        # Face lost tracking
        self.last_face_time = 0.0

        # Frame size
        self.frame_width = 0
        self.frame_height = 0

    def _init_camera_matrix(self, width: int, height: int):
        """Initialize camera matrix based on frame dimensions"""
        if self.frame_width != width or self.frame_height != height:
            self.frame_width = width
            self.frame_height = height

            # Approximate camera matrix
            focal_length = width
            center = (width / 2, height / 2)

            self.camera_matrix = np.array([
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1]
            ], dtype=np.float64)

    def _get_landmark_points(self, landmarks, width: int, height: int) -> np.ndarray:
        """Extract 2D points from face landmarks"""
        points = []
        for idx in self.LANDMARK_INDICES:
            lm = landmarks.landmark[idx]
            x = lm.x * width
            y = lm.y * height
            points.append((x, y))
        return np.array(points, dtype=np.float64)

    def _estimate_pose(self, image_points: np.ndarray) -> Tuple[float, float, float]:
        """
        Estimate head pose using solvePnP.
        Returns (yaw, pitch, roll) in degrees.
        """
        # Solve PnP
        success, rotation_vec, translation_vec = cv2.solvePnP(
            self.MODEL_POINTS,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            return 0.0, 0.0, 0.0

        # Convert rotation vector to rotation matrix
        rotation_mat, _ = cv2.Rodrigues(rotation_vec)

        # Get Euler angles from rotation matrix
        # Using the decomposition: R = Rz * Ry * Rx
        sy = np.sqrt(rotation_mat[0, 0] ** 2 + rotation_mat[1, 0] ** 2)

        if sy > 1e-6:
            pitch = np.arctan2(-rotation_mat[2, 0], sy)
            yaw = np.arctan2(rotation_mat[1, 0], rotation_mat[0, 0])
            roll = np.arctan2(rotation_mat[2, 1], rotation_mat[2, 2])
        else:
            pitch = np.arctan2(-rotation_mat[2, 0], sy)
            yaw = np.arctan2(-rotation_mat[1, 2], rotation_mat[1, 1])
            roll = 0

        # Convert to degrees
        yaw_deg = np.degrees(yaw)
        pitch_deg = np.degrees(pitch)
        roll_deg = np.degrees(roll)

        return yaw_deg, pitch_deg, roll_deg

    def _apply_smoothing(self, yaw: float, pitch: float, roll: float):
        """Apply exponential smoothing to pose values"""
        alpha = 1.0 - config.SMOOTHING_FACTOR

        self.smoothed_yaw = alpha * yaw + config.SMOOTHING_FACTOR * self.smoothed_yaw
        self.smoothed_pitch = alpha * pitch + config.SMOOTHING_FACTOR * self.smoothed_pitch
        self.smoothed_roll = alpha * roll + config.SMOOTHING_FACTOR * self.smoothed_roll

        return self.smoothed_yaw, self.smoothed_pitch, self.smoothed_roll

    def calibrate(self, frame: np.ndarray) -> bool:
        """
        Calibrate neutral head position.
        Call this when the user is looking straight at the camera.
        Returns True if calibration successful.
        """
        pose = self.process_frame(frame, apply_calibration=False)

        if pose.face_detected:
            self.neutral_yaw = pose.yaw
            self.neutral_pitch = pose.pitch
            self.is_calibrated = True

            if config.DEBUG_CONSOLE:
                print(f"[CALIBRATION] Neutral position set: yaw={self.neutral_yaw:.1f}, pitch={self.neutral_pitch:.1f}")

            return True

        return False

    def reset_calibration(self):
        """Reset calibration to default values"""
        self.neutral_yaw = config.NEUTRAL_YAW
        self.neutral_pitch = config.NEUTRAL_PITCH
        self.is_calibrated = False

        if config.DEBUG_CONSOLE:
            print("[CALIBRATION] Reset to default")

    def process_frame(self, frame: np.ndarray, apply_calibration: bool = True) -> HeadPose:
        """
        Process a video frame and extract head pose.

        Args:
            frame: BGR image from camera
            apply_calibration: If True, subtract neutral position

        Returns:
            HeadPose with yaw, pitch, roll values
        """
        height, width = frame.shape[:2]
        self._init_camera_matrix(width, height)

        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process with Face Mesh
        results = self.face_mesh.process(rgb_frame)

        current_time = time.time()

        if results.multi_face_landmarks:
            # Get first face
            face_landmarks = results.multi_face_landmarks[0]

            # Extract key points
            image_points = self._get_landmark_points(face_landmarks, width, height)

            # Estimate pose
            yaw, pitch, roll = self._estimate_pose(image_points)

            # Apply smoothing
            yaw, pitch, roll = self._apply_smoothing(yaw, pitch, roll)

            # Apply calibration offset
            if apply_calibration:
                yaw -= self.neutral_yaw
                pitch -= self.neutral_pitch

            # Update pose
            self.current_pose = HeadPose(
                yaw=yaw,
                pitch=pitch,
                roll=roll,
                face_detected=True,
                timestamp=current_time
            )

            self.last_face_time = current_time
        else:
            # No face detected
            # Check if we've lost face for too long
            if current_time - self.last_face_time > config.FACE_LOST_TIMEOUT:
                # Reset to center
                self.current_pose = HeadPose(
                    yaw=0.0,
                    pitch=0.0,
                    roll=0.0,
                    face_detected=False,
                    timestamp=current_time
                )
            else:
                # Keep last pose but mark as not detected
                self.current_pose.face_detected = False
                self.current_pose.timestamp = current_time

        return self.current_pose

    def draw_debug(self, frame: np.ndarray, pose: HeadPose) -> np.ndarray:
        """
        Draw debug overlay on frame.

        Args:
            frame: BGR image
            pose: HeadPose data

        Returns:
            Frame with debug overlay
        """
        output = frame.copy()
        height, width = frame.shape[:2]

        # Convert to RGB for processing
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb_frame)

        # Draw face mesh
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # Draw face mesh contours
                self.mp_drawing.draw_landmarks(
                    image=output,
                    landmark_list=face_landmarks,
                    connections=self.mp_face_mesh.FACEMESH_CONTOURS,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_contours_style()
                )

                # Draw key points used for pose estimation
                for idx in self.LANDMARK_INDICES:
                    lm = face_landmarks.landmark[idx]
                    x = int(lm.x * width)
                    y = int(lm.y * height)
                    cv2.circle(output, (x, y), 5, (0, 255, 0), -1)

        # Draw pose info
        status_color = (0, 255, 0) if pose.face_detected else (0, 0, 255)
        status_text = "TRACKING" if pose.face_detected else "NO FACE"

        cv2.putText(output, f"Status: {status_text}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        cv2.putText(output, f"Yaw: {pose.yaw:.1f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(output, f"Pitch: {pose.pitch:.1f}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(output, f"Roll: {pose.roll:.1f}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Draw calibration status
        cal_text = "CALIBRATED" if self.is_calibrated else "NOT CALIBRATED"
        cal_color = (0, 255, 0) if self.is_calibrated else (0, 165, 255)
        cv2.putText(output, f"Cal: {cal_text}", (10, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, cal_color, 1)

        # Draw direction indicator (crosshair)
        center_x = width // 2
        center_y = height // 2

        # Map pose to crosshair offset
        offset_x = int(pose.yaw / config.HEAD_YAW_RANGE * (width // 4))
        offset_y = int(-pose.pitch / config.HEAD_PITCH_RANGE * (height // 4))

        # Clamp offsets
        offset_x = max(-width // 4, min(width // 4, offset_x))
        offset_y = max(-height // 4, min(height // 4, offset_y))

        # Draw crosshair
        crosshair_x = center_x + offset_x
        crosshair_y = center_y + offset_y

        cv2.line(output, (crosshair_x - 20, crosshair_y), (crosshair_x + 20, crosshair_y), (0, 255, 255), 2)
        cv2.line(output, (crosshair_x, crosshair_y - 20), (crosshair_x, crosshair_y + 20), (0, 255, 255), 2)
        cv2.circle(output, (crosshair_x, crosshair_y), 10, (0, 255, 255), 2)

        # Draw center reference
        cv2.circle(output, (center_x, center_y), 5, (128, 128, 128), -1)

        return output

    def close(self):
        """Release resources"""
        self.face_mesh.close()


# Test standalone
if __name__ == "__main__":
    print("Testing HeadTracker with webcam...")

    cap = cv2.VideoCapture(0)
    tracker = HeadTracker()

    print("Press 'c' to calibrate, 'r' to reset, 'q' to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Process frame
        pose = tracker.process_frame(frame)

        # Draw debug
        debug_frame = tracker.draw_debug(frame, pose)

        # Show
        cv2.imshow("Head Tracker Test", debug_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            tracker.calibrate(frame)
        elif key == ord('r'):
            tracker.reset_calibration()

    cap.release()
    cv2.destroyAllWindows()
    tracker.close()
