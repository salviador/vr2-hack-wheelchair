# Virtual Joy Raspberry Pi

Head-tracking virtual joystick controller for VR2 wheelchair using Raspberry Pi Camera 3.

## Overview

This software uses head tracking to control a wheelchair via the VR2 protocol:
- **Turn head left/right** -> Steer left/right (X axis)
- **Tilt head up/down** -> Move forward/backward (Y axis)

Communicates with `joy_vr2_emulator` (ESP32) via BLE.

## Hardware Requirements

- Raspberry Pi 4/5 (recommended) or any Linux PC
- Raspberry Pi Camera 3 or USB webcam
- Bluetooth adapter (built-in on Pi 4/5)
- `joy_vr2_emulator` device (ESP32)

## Installation

### On Raspberry Pi

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install system dependencies
sudo apt install -y python3-pip python3-opencv python3-picamera2 libcap-dev

# Install Python dependencies
cd Virtual_Joy_raspberry
pip3 install -r requirements.txt
```

### On Linux PC (for testing)

```bash
# Install dependencies
sudo apt install -y python3-pip python3-opencv bluetooth bluez

# Install Python dependencies
pip3 install -r requirements.txt
```

## Usage

### Basic usage (with BLE and display)

```bash
python3 main.py
```

### Test mode (no BLE connection)

```bash
python3 main.py --no-ble
```

### Force USB webcam

```bash
python3 main.py --webcam
```

### Headless mode (no display)

```bash
python3 main.py --no-display
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| `c` | Calibrate neutral head position |
| `r` | Reset calibration to default |
| `b` | Reconnect BLE |
| `s` | Send START command to ECU |
| `x` | Send SHUTDOWN command |
| `+` | Increase speed level |
| `-` | Decrease speed level |
| `p` | Pause/resume joystick output |
| `q` | Quit |

## Calibration

1. Start the application
2. Position your head in the neutral/center position
3. Press `c` to calibrate
4. The system will now use this position as the center point

## Configuration

Edit `config.py` to customize:

### Head Tracking
- `HEAD_YAW_RANGE`: Degrees of rotation for full X range (default: 30)
- `HEAD_PITCH_RANGE`: Degrees of tilt for full Y range (default: 25)
- `DEAD_ZONE_YAW/PITCH`: Dead zone in degrees (default: 3)
- `SMOOTHING_FACTOR`: Movement smoothing (0-1, default: 0.3)

### Safety
- `MAX_OUTPUT_PERCENT`: Maximum speed percentage (default: 80%)
- `FACE_LOST_TIMEOUT`: Stop if face lost for X seconds (default: 0.5)

### Camera
- `CAMERA_WIDTH/HEIGHT`: Resolution (default: 640x480)
- `USE_LIBCAMERA`: Use Pi Camera (True) or USB webcam (False)

### BLE
- `BLE_DEVICE_NAME`: Device name to scan for ("WHEELS Mic")
- `BLE_UPDATE_RATE`: Commands per second (default: 20 Hz)

## File Structure

```
Virtual_Joy_raspberry/
├── main.py           # Main application entry point
├── config.py         # Configuration settings
├── head_tracker.py   # Face detection and head pose estimation
├── joy_mapper.py     # Maps head pose to joystick coordinates
├── ble_client.py     # BLE communication with wheelchair
├── requirements.txt  # Python dependencies
└── README.md         # This file
```

## BLE Protocol

Commands sent to `joy_vr2_emulator`:

| Command | Hex | Format |
|---------|-----|--------|
| SET_JOY | 0x01 | `[0x01][X][Y][BTN][CKSUM]` |
| START | 0x02 | `[0x02][CKSUM]` |
| SHUTDOWN | 0x03 | `[0x03][CKSUM]` |
| SPEED+ | 0x04 | `[0x04][CKSUM]` |
| SPEED- | 0x05 | `[0x05][CKSUM]` |

- X, Y: signed int8 (-100 to +100)
- Checksum: sum of bytes XOR 0xFF

## Troubleshooting

### Camera not detected
- Check if camera is connected: `libcamera-hello` (Pi) or `ls /dev/video*`
- Try `--webcam` flag to force USB camera
- Check permissions: `sudo usermod -a -G video $USER`

### BLE not connecting
- Check Bluetooth is enabled: `sudo systemctl status bluetooth`
- Scan for devices: `bluetoothctl scan on`
- Ensure `joy_vr2_emulator` is powered and advertising

### Poor tracking
- Ensure good lighting on your face
- Adjust `FACE_DETECTION_CONFIDENCE` in config
- Calibrate in your typical usage position
- Reduce `SMOOTHING_FACTOR` for faster response

### High CPU usage
- Reduce `CAMERA_WIDTH/HEIGHT`
- Reduce `CAMERA_FPS`
- Run headless with `--no-display`

## Testing Individual Modules

```bash
# Test head tracker
python3 head_tracker.py

# Test joy mapper
python3 joy_mapper.py

# Test BLE client
python3 ble_client.py
```

## License

MIT License - See main project for details.
