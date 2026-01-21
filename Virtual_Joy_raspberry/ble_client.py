"""
BLE Client Module for VR2 Wheelchair Communication
Uses bleak library for BLE communication on Raspberry Pi
"""

import asyncio
import struct
from dataclasses import dataclass
from typing import Optional, Callable
from enum import IntEnum

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError

import config


class WheelchairState(IntEnum):
    """ECU state from telemetry"""
    IDLE = 0
    INIT = 1
    RUN = 2
    ERROR = 3
    SHUTDOWN = 4


@dataclass
class Telemetry:
    """Telemetry data from wheelchair"""
    state: WheelchairState = WheelchairState.IDLE
    speed_level: int = 1
    battery_code: int = 0
    switches: int = 0
    raw_data: bytes = b''


class BLEClient:
    """
    BLE Client for VR2 Wheelchair communication.

    Connects to the joy_vr2_emulator device and sends joystick commands.
    Receives telemetry data via notifications.
    """

    def __init__(self, on_telemetry: Optional[Callable[[Telemetry], None]] = None):
        """
        Initialize BLE client.

        Args:
            on_telemetry: Callback function for telemetry updates
        """
        self.device_name = config.BLE_DEVICE_NAME
        self.service_uuid = config.BLE_SERVICE_UUID
        self.joytx_uuid = config.BLE_CHAR_JOYTX_UUID
        self.telemetry_uuid = config.BLE_CHAR_TELEMETRY_UUID

        self.client: Optional[BleakClient] = None
        self.device_address: Optional[str] = None
        self.connected = False

        self.on_telemetry = on_telemetry
        self.last_telemetry = Telemetry()

        # Connection state
        self._reconnect_task = None
        self._running = False

    @staticmethod
    def _calc_checksum(data: bytes) -> int:
        """Calculate checksum: sum of bytes XOR 0xFF"""
        checksum = sum(data) & 0xFF
        return checksum ^ 0xFF

    @staticmethod
    def _verify_checksum(data: bytes) -> bool:
        """Verify checksum of received data"""
        if len(data) < 2:
            return False
        calc = BLEClient._calc_checksum(data[:-1])
        return calc == data[-1]

    def _build_joystick_command(self, x: int, y: int, buttons: int = 0) -> bytes:
        """
        Build SET_JOY command packet.

        Args:
            x: X axis value (-100 to +100)
            y: Y axis value (-100 to +100)
            buttons: Button flags

        Returns:
            Command bytes with checksum
        """
        # Clamp values
        x = max(-100, min(100, x))
        y = max(-100, min(100, y))
        buttons = buttons & 0xFF

        # Build command: [CMD][X][Y][BUTTONS][CHECKSUM]
        # X and Y are signed 8-bit integers
        cmd_data = struct.pack('bbb', config.BLE_CMD_SET_JOY, x, y)
        cmd_data += struct.pack('B', buttons)
        checksum = self._calc_checksum(cmd_data)
        cmd_data += struct.pack('B', checksum)

        return cmd_data

    def _build_simple_command(self, cmd: int) -> bytes:
        """
        Build a simple command packet (no data).

        Args:
            cmd: Command byte (START, SHUTDOWN, SPEED_PLUS, SPEED_MINUS)

        Returns:
            Command bytes with checksum
        """
        cmd_data = struct.pack('B', cmd)
        checksum = self._calc_checksum(cmd_data)
        cmd_data += struct.pack('B', checksum)
        return cmd_data

    def _parse_telemetry(self, data: bytes) -> Optional[Telemetry]:
        """Parse telemetry notification data"""
        if len(data) < 5:
            return None

        if not self._verify_checksum(data):
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Invalid telemetry checksum: {data.hex()}")
            return None

        state = data[0]
        speed = data[1]
        battery = data[2]
        switches = data[3]

        try:
            state_enum = WheelchairState(state)
        except ValueError:
            state_enum = WheelchairState.IDLE

        return Telemetry(
            state=state_enum,
            speed_level=speed,
            battery_code=battery,
            switches=switches,
            raw_data=data
        )

    def _telemetry_handler(self, sender, data: bytearray):
        """Handle telemetry notification"""
        telemetry = self._parse_telemetry(bytes(data))
        if telemetry:
            self.last_telemetry = telemetry
            if self.on_telemetry:
                self.on_telemetry(telemetry)

    async def scan(self, timeout: float = 10.0) -> Optional[str]:
        """
        Scan for wheelchair device.

        Args:
            timeout: Scan timeout in seconds

        Returns:
            Device address if found, None otherwise
        """
        if config.DEBUG_CONSOLE:
            print(f"[BLE] Scanning for '{self.device_name}'...")

        devices = await BleakScanner.discover(timeout=timeout)

        for device in devices:
            if device.name and self.device_name in device.name:
                if config.DEBUG_CONSOLE:
                    print(f"[BLE] Found device: {device.name} ({device.address})")
                self.device_address = device.address
                return device.address

        if config.DEBUG_CONSOLE:
            print(f"[BLE] Device not found")
        return None

    async def connect(self, address: Optional[str] = None) -> bool:
        """
        Connect to wheelchair device.

        Args:
            address: Device address (will scan if not provided)

        Returns:
            True if connected successfully
        """
        if address:
            self.device_address = address
        elif not self.device_address:
            await self.scan()

        if not self.device_address:
            return False

        try:
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Connecting to {self.device_address}...")

            self.client = BleakClient(self.device_address)
            await self.client.connect()

            if self.client.is_connected:
                self.connected = True
                if config.DEBUG_CONSOLE:
                    print(f"[BLE] Connected!")

                # Subscribe to telemetry notifications
                try:
                    await self.client.start_notify(
                        self.telemetry_uuid,
                        self._telemetry_handler
                    )
                    if config.DEBUG_CONSOLE:
                        print(f"[BLE] Subscribed to telemetry")
                except Exception as e:
                    if config.DEBUG_CONSOLE:
                        print(f"[BLE] Could not subscribe to telemetry: {e}")

                return True

        except BleakError as e:
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Connection error: {e}")
            self.connected = False

        return False

    async def disconnect(self):
        """Disconnect from device"""
        if self.client and self.client.is_connected:
            try:
                await self.client.stop_notify(self.telemetry_uuid)
            except Exception:
                pass

            await self.client.disconnect()
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Disconnected")

        self.connected = False
        self.client = None

    async def send_joystick(self, x: int, y: int, buttons: int = 0) -> bool:
        """
        Send joystick command.

        Args:
            x: X axis (-100 to +100)
            y: Y axis (-100 to +100)
            buttons: Button flags

        Returns:
            True if sent successfully
        """
        if not self.connected or not self.client:
            return False

        cmd = self._build_joystick_command(x, y, buttons)

        try:
            await self.client.write_gatt_char(
                self.joytx_uuid,
                cmd,
                response=False
            )
            return True
        except BleakError as e:
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Send error: {e}")
            self.connected = False
            return False

    async def send_start(self) -> bool:
        """Send START command to initialize VR2"""
        if not self.connected or not self.client:
            return False

        cmd = self._build_simple_command(config.BLE_CMD_START)

        try:
            await self.client.write_gatt_char(self.joytx_uuid, cmd, response=False)
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Sent START command")
            return True
        except BleakError as e:
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Send error: {e}")
            return False

    async def send_shutdown(self) -> bool:
        """Send SHUTDOWN command"""
        if not self.connected or not self.client:
            return False

        cmd = self._build_simple_command(config.BLE_CMD_SHUTDOWN)

        try:
            await self.client.write_gatt_char(self.joytx_uuid, cmd, response=False)
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Sent SHUTDOWN command")
            return True
        except BleakError as e:
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Send error: {e}")
            return False

    async def send_speed_plus(self) -> bool:
        """Increase speed level"""
        if not self.connected or not self.client:
            return False

        cmd = self._build_simple_command(config.BLE_CMD_SPEED_PLUS)

        try:
            await self.client.write_gatt_char(self.joytx_uuid, cmd, response=False)
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Sent SPEED+ command")
            return True
        except BleakError as e:
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Send error: {e}")
            return False

    async def send_speed_minus(self) -> bool:
        """Decrease speed level"""
        if not self.connected or not self.client:
            return False

        cmd = self._build_simple_command(config.BLE_CMD_SPEED_MINUS)

        try:
            await self.client.write_gatt_char(self.joytx_uuid, cmd, response=False)
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Sent SPEED- command")
            return True
        except BleakError as e:
            if config.DEBUG_CONSOLE:
                print(f"[BLE] Send error: {e}")
            return False

    def get_state_name(self) -> str:
        """Get current wheelchair state as string"""
        return self.last_telemetry.state.name

    def get_battery_voltage(self) -> float:
        """
        Decode battery voltage from battery code.
        This is an approximation based on common VR2 battery codes.
        """
        # Battery codes roughly map to voltages
        # 0x12 = ~10V (low), 0xA1 = ~28V (full)
        code = self.last_telemetry.battery_code
        if code == 0:
            return 0.0
        # Linear approximation
        voltage = 10.0 + (code - 0x12) * (28.0 - 10.0) / (0xA1 - 0x12)
        return max(0.0, min(30.0, voltage))


class BLEClientSync:
    """
    Synchronous wrapper for BLEClient.
    Easier to use in synchronous code.
    """

    def __init__(self, on_telemetry: Optional[Callable[[Telemetry], None]] = None):
        self._client = BLEClient(on_telemetry)
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def _get_loop(self):
        if self._loop is None or self._loop.is_closed():
            try:
                self._loop = asyncio.get_event_loop()
            except RuntimeError:
                self._loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self._loop)
        return self._loop

    def scan(self, timeout: float = 10.0) -> Optional[str]:
        return self._get_loop().run_until_complete(self._client.scan(timeout))

    def connect(self, address: Optional[str] = None) -> bool:
        return self._get_loop().run_until_complete(self._client.connect(address))

    def disconnect(self):
        self._get_loop().run_until_complete(self._client.disconnect())

    def send_joystick(self, x: int, y: int, buttons: int = 0) -> bool:
        return self._get_loop().run_until_complete(self._client.send_joystick(x, y, buttons))

    def send_start(self) -> bool:
        return self._get_loop().run_until_complete(self._client.send_start())

    def send_shutdown(self) -> bool:
        return self._get_loop().run_until_complete(self._client.send_shutdown())

    def send_speed_plus(self) -> bool:
        return self._get_loop().run_until_complete(self._client.send_speed_plus())

    def send_speed_minus(self) -> bool:
        return self._get_loop().run_until_complete(self._client.send_speed_minus())

    @property
    def connected(self) -> bool:
        return self._client.connected

    @property
    def last_telemetry(self) -> Telemetry:
        return self._client.last_telemetry

    def get_state_name(self) -> str:
        return self._client.get_state_name()

    def get_battery_voltage(self) -> float:
        return self._client.get_battery_voltage()


# Test standalone
if __name__ == "__main__":
    import time

    def on_telemetry(tele: Telemetry):
        print(f"[TELEMETRY] State: {tele.state.name}, Speed: {tele.speed_level}, "
              f"Battery: 0x{tele.battery_code:02X}, Switches: 0x{tele.switches:02X}")

    print("Testing BLE Client...")
    print("=" * 50)

    client = BLEClientSync(on_telemetry)

    # Scan for device
    address = client.scan(timeout=10)

    if address:
        print(f"\nFound device at: {address}")

        # Connect
        if client.connect(address):
            print("Connected!\n")

            # Send start command
            client.send_start()
            time.sleep(0.5)

            # Test joystick movements
            test_moves = [
                (0, 0, "Center"),
                (50, 0, "Right"),
                (-50, 0, "Left"),
                (0, 50, "Forward"),
                (0, -50, "Backward"),
                (0, 0, "Stop"),
            ]

            for x, y, name in test_moves:
                print(f"Sending: {name} (x={x}, y={y})")
                client.send_joystick(x, y)
                time.sleep(0.5)

            # Disconnect
            client.disconnect()
            print("\nDisconnected")
        else:
            print("Failed to connect")
    else:
        print("Device not found")

    print("=" * 50)
    print("Test complete")
