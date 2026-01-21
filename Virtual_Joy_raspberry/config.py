"""
Configuration file for Virtual Joy Raspberry Pi Head Tracker
"""

# =============================================================================
# BLE CONFIGURATION - VR2 Wheelchair Protocol
# =============================================================================

# BLE Device name to scan for
BLE_DEVICE_NAME = "WHEELS Mic"

# BLE Service and Characteristic UUIDs (from joy_vr2_emulator)
BLE_SERVICE_UUID = "da343711-d712-3230-8842-f9dea033233d"
BLE_CHAR_JOYTX_UUID = "da343711-d712-3230-8842-f7dea033233d"  # Write - commands to ESP32
BLE_CHAR_JOYRX_UUID = "da343711-d712-3230-8842-f8dea033233d"  # Notify - backward compat
BLE_CHAR_TELEMETRY_UUID = "da343711-d712-3230-8842-f6dea033233d"  # Notify - telemetry

# BLE Commands
BLE_CMD_SET_JOY = 0x01
BLE_CMD_START = 0x02
BLE_CMD_SHUTDOWN = 0x03
BLE_CMD_SPEED_PLUS = 0x04
BLE_CMD_SPEED_MINUS = 0x05

# BLE Button flags
BLE_BTN_NONE = 0x00
BLE_BTN_BRAKE = 0x01       # Z button
BLE_BTN_BOOST = 0x02       # C button
BLE_BTN_SPEED_MINUS = 0x04
BLE_BTN_SPEED_PLUS = 0x08

# Joystick range limits (VR2 protocol)
JOY_MIN = -100
JOY_MAX = 100
JOY_CENTER = 0

# BLE update rate (Hz)
BLE_UPDATE_RATE = 20  # 50ms interval

# =============================================================================
# CAMERA CONFIGURATION - Raspberry Pi Camera 3
# =============================================================================

# Camera resolution
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

# Camera rotation (0, 90, 180, 270)
CAMERA_ROTATION = 0

# Use libcamera (True) or legacy picamera (False)
USE_LIBCAMERA = True

# =============================================================================
# HEAD TRACKING CONFIGURATION
# =============================================================================

# MediaPipe Face Mesh settings
FACE_DETECTION_CONFIDENCE = 0.5
FACE_TRACKING_CONFIDENCE = 0.5
MAX_NUM_FACES = 1

# Head pose neutral position (calibrated on startup)
# These are default values, will be calibrated
NEUTRAL_YAW = 0.0    # Left/Right rotation (degrees)
NEUTRAL_PITCH = 0.0  # Up/Down rotation (degrees)

# Head movement range (degrees from neutral)
HEAD_YAW_RANGE = 30.0    # +/- degrees for full joystick X range
HEAD_PITCH_RANGE = 25.0  # +/- degrees for full joystick Y range

# Dead zone (degrees) - no movement if within this range
DEAD_ZONE_YAW = 3.0
DEAD_ZONE_PITCH = 3.0

# Smoothing factor (0.0 = no smoothing, 1.0 = max smoothing)
# Higher values = smoother but slower response
SMOOTHING_FACTOR = 0.3

# Invert axes
INVERT_X = False  # True to invert left/right
INVERT_Y = False  # True to invert forward/backward

# =============================================================================
# SAFETY CONFIGURATION
# =============================================================================

# Face lost timeout (seconds) - stop joystick if face not detected
FACE_LOST_TIMEOUT = 0.5

# Maximum joystick output (percentage of JOY_MAX)
# Use this to limit maximum speed for safety
MAX_OUTPUT_PERCENT = 80  # 80% of max speed

# Enable/disable joystick output (for testing)
JOYSTICK_ENABLED = True

# =============================================================================
# DEBUG CONFIGURATION
# =============================================================================

# Show debug window with camera feed and overlay
DEBUG_DISPLAY = True

# Print debug info to console
DEBUG_CONSOLE = True

# Log to file
DEBUG_LOG_FILE = None  # Set to filename to enable, e.g., "debug.log"

# FPS display update interval (seconds)
FPS_UPDATE_INTERVAL = 1.0
