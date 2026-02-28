#!/usr/bin/env python3
"""
vr2_bridge.py — VR2 Wheelchair BLE ↔ MQTT Bridge
==================================================
Raspberry Pi 5 bridge between ESP32-S3 (BLE) and Mosquitto MQTT.
Designed as integration point for ROS2 nodes and development tools.

Architecture:
    ESP32-S3 (BLE GATT) ←─bleak──► [this bridge] ←─paho-mqtt──► Mosquitto
                                                                       │
                                                               ROS2 nodes / apps

═══════════════════════════════════════════════════════════════════════════════
MQTT Topics
═══════════════════════════════════════════════════════════════════════════════

  SUBSCRIBE (commands IN):
    vr2/power          "on"  → sends START  to ECU
                       "off" → sends SHUTDOWN to ECU

    vr2/set_joystick   JSON: {"x": -100..+100, "y": -100..+100}
                         x > 0 = right,  x < 0 = left
                         y > 0 = forward, y < 0 = backward

  PUBLISH (state OUT):
    vr2/connected      "true" | "false"   (retained)
    vr2/state          "IDLE" | "INIT" | "RUN" | "ERROR" | "SHUTDOWN"
    vr2/speed          "1" .. "5"
    vr2/switch         JSON: {"sw2c": bool, "swl2": bool, "sw2r": bool,
                               "sw2dw": bool, "sw2up": bool}
                         sw2c  = GPIO0   sw2r  = GPIO35
                         swl2  = GPIO45  sw2dw = GPIO37  sw2up = GPIO36

═══════════════════════════════════════════════════════════════════════════════
Setup
═══════════════════════════════════════════════════════════════════════════════

    pip install bleak paho-mqtt
    pip install dbus-next          # optional — enables automatic BLE pairing

  First-time BLE pairing (if dbus-next not installed):
    sudo bluetoothctl
    > agent on
    > scan on                      # find "WHEELS Mic"
    > pair <MAC>                   # enter PIN when prompted: 847293
    > trust <MAC>
    > quit

  Run as service (systemd):
    copy vr2_bridge.service to /etc/systemd/system/
    systemctl enable --now vr2_bridge

═══════════════════════════════════════════════════════════════════════════════
"""

import asyncio
import json
import logging
import signal
import threading
from typing import Optional

import paho.mqtt.client as mqtt
from bleak import BleakClient, BleakScanner

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION  ← edit these values
# ─────────────────────────────────────────────────────────────────────────────

# BLE
BLE_DEVICE_NAME      = "WHEELS Mic"   # ESP32-S3 advertising name (bleService.c)
BLE_PIN              = 847293          # Pairing PIN — must match ESP32 passkey in bleService.c
BLE_SCAN_TIMEOUT     = 15.0           # seconds for each scan attempt
BLE_RECONNECT_DELAY  = 5.0            # seconds between reconnect attempts

# BLE UUIDs — must match bleService.c exactly
SERVICE_UUID         = "da343711-d712-3230-8842-f9dea033233d"
CHAR_WRITE_UUID      = "da343711-d712-3230-8842-f7dea033233d"   # write commands here
CHAR_TELEMETRY_UUID  = "da343711-d712-3230-8842-f6dea033233d"   # telemetry notifications

# MQTT
MQTT_BROKER    = "localhost"
MQTT_PORT      = 1883
MQTT_KEEPALIVE = 60
MQTT_CLIENT_ID = "vr2_bridge"

# MQTT topics — subscribe
TOPIC_POWER    = "vr2/power"
TOPIC_JOYSTICK = "vr2/set_joystick"

# MQTT topics — publish
TOPIC_CONNECTED = "vr2/connected"
TOPIC_STATE     = "vr2/state"
TOPIC_SPEED     = "vr2/speed"
TOPIC_SWITCH    = "vr2/switch"

# ─────────────────────────────────────────────────────────────────────────────
# BLE PROTOCOL CONSTANTS  (mirrors bleService.h)
# ─────────────────────────────────────────────────────────────────────────────

CMD_SET_JOY     = 0x01
CMD_START       = 0x02
CMD_SHUTDOWN    = 0x03
CMD_SPEED_PLUS  = 0x04
CMD_SPEED_MINUS = 0x05

BTN_NONE        = 0x00
BTN_BRAKE       = 0x01
BTN_BOOST       = 0x02

ECU_STATES = {
    0x00: "IDLE",
    0x01: "INIT",
    0x02: "RUN",
    0x03: "ERROR",
    0x04: "SHUTDOWN",
}

# ECU speed byte → human level 1-5
SPEED_MAP = {0x21: 1, 0x41: 2, 0x61: 3, 0x81: 4, 0xA1: 5}

# Telemetry switches bitmask (byte 3)
SW_SW2C  = 0x01   # GPIO0
SW_SWL2  = 0x02   # GPIO45
SW_SW2R  = 0x04   # GPIO35
SW_SW2DW = 0x08   # GPIO37
SW_SW2UP = 0x10   # GPIO36

# ─────────────────────────────────────────────────────────────────────────────
# LOGGING
# ─────────────────────────────────────────────────────────────────────────────

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("vr2_bridge")

# ─────────────────────────────────────────────────────────────────────────────
# BLUEZ PAIRING AGENT  (optional — requires: pip install dbus-next)
#
# Needed only for the FIRST connection when the device is not yet bonded.
# After first pairing, BlueZ remembers the bond and reconnects automatically.
# ─────────────────────────────────────────────────────────────────────────────

_DBUS_AVAILABLE = False
try:
    from dbus_next.aio import MessageBus
    from dbus_next import BusType
    from dbus_next.service import ServiceInterface, method as dbus_method
    _DBUS_AVAILABLE = True
except ImportError:
    pass

_AGENT_PATH = "/vr2/bridge/agent"

if _DBUS_AVAILABLE:
    class _PairingAgent(ServiceInterface):
        """
        BlueZ Agent1 implementation.
        Auto-supplies the pairing PIN when ESP32 requests it.

        ESP32 uses IO_CAP_OUT (Display Only) + SC_MITM_BOND:
          → BlueZ calls RequestPasskey() on us  → we return BLE_PIN
        """

        def __init__(self, passkey: int):
            super().__init__("org.bluez.Agent1")
            self._passkey = passkey

        @dbus_method()
        async def Release(self) -> None:
            pass

        @dbus_method()
        async def RequestPinCode(self, device: "o") -> "s":
            log.info(f"[Pairing] RequestPinCode ← {device}")
            return str(self._passkey)

        @dbus_method()
        async def DisplayPinCode(self, device: "o", pincode: "s") -> None:
            log.info(f"[Pairing] DisplayPinCode: {pincode}")

        @dbus_method()
        async def RequestPasskey(self, device: "o") -> "u":
            log.info(f"[Pairing] RequestPasskey ← {device} → returning {self._passkey}")
            return self._passkey  # uint32

        @dbus_method()
        async def DisplayPasskey(self, device: "o", passkey: "u", entered: "q") -> None:
            log.info(f"[Pairing] DisplayPasskey: {passkey} (entered: {entered})")

        @dbus_method()
        async def RequestConfirmation(self, device: "o", passkey: "u") -> None:
            log.info(f"[Pairing] RequestConfirmation: {passkey} — auto-confirming")
            # No exception = accepted

        @dbus_method()
        async def RequestAuthorization(self, device: "o") -> None:
            log.info(f"[Pairing] RequestAuthorization ← {device} — auto-authorizing")

        @dbus_method()
        async def AuthorizeService(self, device: "o", uuid: "s") -> None:
            log.info(f"[Pairing] AuthorizeService: {uuid} — auto-authorizing")

        @dbus_method()
        async def Cancel(self) -> None:
            log.warning("[Pairing] Cancelled by BlueZ")


async def _register_bluez_agent(passkey: int):
    """
    Register a BlueZ pairing agent on the system D-Bus.
    Returns the bus object (keep alive) or None if unavailable.
    """
    if not _DBUS_AVAILABLE:
        log.warning(
            "dbus-next not installed — automatic BLE pairing disabled.\n"
            "  Install:  pip install dbus-next\n"
            "  Manual pairing (one-time):\n"
            "    bluetoothctl → agent on → scan on → pair <MAC>\n"
            "    enter PIN: %d → trust <MAC> → quit",
            passkey,
        )
        return None

    try:
        bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
        agent = _PairingAgent(passkey)
        bus.export(_AGENT_PATH, agent)

        intro  = await bus.introspect("org.bluez", "/org/bluez")
        proxy  = bus.get_proxy_object("org.bluez", "/org/bluez", intro)
        mgr    = proxy.get_interface("org.bluez.AgentManager1")

        # KeyboardDisplay: BlueZ will call RequestPasskey() when pairing
        await mgr.call_register_agent(_AGENT_PATH, "KeyboardDisplay")
        await mgr.call_request_default_agent(_AGENT_PATH)

        log.info("BlueZ pairing agent registered — auto-PIN enabled (PIN=%d)", passkey)
        return bus

    except Exception as exc:
        log.warning(
            "Could not register BlueZ pairing agent: %s\n"
            "  Manual pairing may be required on first connect.",
            exc,
        )
        return None


# ─────────────────────────────────────────────────────────────────────────────
# BLE PROTOCOL HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def _ble_checksum(data: bytes) -> int:
    """Checksum = (sum of bytes) XOR 0xFF  — matches ESP32 ble_calc_checksum()"""
    return (sum(data) & 0xFF) ^ 0xFF


def _build_cmd(*args: int) -> bytes:
    """Build a BLE command frame with appended checksum."""
    payload = bytes(args)
    return payload + bytes([_ble_checksum(payload)])


def cmd_start() -> bytes:
    return _build_cmd(CMD_START)


def cmd_shutdown() -> bytes:
    return _build_cmd(CMD_SHUTDOWN)


def cmd_speed_plus() -> bytes:
    return _build_cmd(CMD_SPEED_PLUS)


def cmd_speed_minus() -> bytes:
    return _build_cmd(CMD_SPEED_MINUS)


def cmd_joystick(x: int, y: int, buttons: int = BTN_NONE) -> bytes:
    """
    Build SET_JOY command.
      x, y  in range -100 .. +100
      y > 0 = forward,  y < 0 = backward
      x > 0 = right,    x < 0 = left
    """
    x = max(-100, min(100, int(x)))
    y = max(-100, min(100, int(y)))
    # Cast to signed byte: negative values become 0x100 + value (two's complement)
    xb = x & 0xFF
    yb = y & 0xFF
    return _build_cmd(CMD_SET_JOY, xb, yb, buttons & 0xFF)


def parse_telemetry(data: bytearray) -> Optional[dict]:
    """
    Parse 5-byte telemetry frame from ESP32.
    Returns dict or None if checksum fails.
    Frame: [state][speed][battery][switches][checksum]
    """
    if len(data) < 5:
        log.warning("Telemetry frame too short: %d bytes", len(data))
        return None

    state, speed, battery, switches, chk = data[0], data[1], data[2], data[3], data[4]
    expected = _ble_checksum(bytes(data[:4]))

    if expected != chk:
        log.warning("Telemetry checksum mismatch: got 0x%02X expected 0x%02X", chk, expected)
        return None

    return {
        "state":       state,
        "state_name":  ECU_STATES.get(state, f"UNKNOWN_{state:02X}"),
        "speed_raw":   speed,
        "speed":       SPEED_MAP.get(speed, max(1, min(5, (speed >> 5)))),
        "battery":     battery,
        "switches":    switches,
    }


def parse_switches(sw: int) -> dict:
    """Decode the switches byte into a named dict."""
    return {
        "sw2c":  bool(sw & SW_SW2C),   # GPIO0
        "swl2":  bool(sw & SW_SWL2),   # GPIO45
        "sw2r":  bool(sw & SW_SW2R),   # GPIO35
        "sw2dw": bool(sw & SW_SW2DW),  # GPIO37
        "sw2up": bool(sw & SW_SW2UP),  # GPIO36
    }


# ─────────────────────────────────────────────────────────────────────────────
# VR2 BRIDGE
# ─────────────────────────────────────────────────────────────────────────────

class VR2Bridge:
    """
    Main bridge class.

    Thread model:
      ┌─ asyncio event loop ─────────────────────────────────────┐
      │  _ble_loop()        — BLE scan / connect / notify         │
      │  _cmd_sender_task() — drains cmd_queue → BLE write        │
      └──────────────────────────────────────────────────────────┘
      ┌─ MQTT thread (paho loop_start) ──────────────────────────┐
      │  _on_mqtt_message() — enqueues cmds via call_soon_threadsafe │
      └──────────────────────────────────────────────────────────┘
    """

    def __init__(self):
        self._running        = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # BLE state
        self._ble_client: Optional[BleakClient] = None
        self._ble_connected  = False

        # Two separate queues:
        #   _joy_latest : holds at most 1 joystick frame (latest-wins, older discarded)
        #   _cmd_queue  : guaranteed-delivery for critical commands (power, speed)
        self._joy_latest: Optional[bytes] = None   # protected by asyncio (single thread)
        self._cmd_queue  = asyncio.Queue()          # critical commands, never dropped

        # Published state cache — suppress redundant MQTT publishes
        self._last_state     = None
        self._last_speed     = None
        self._last_switch    = None

        # MQTT client
        self._mqtt = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
        self._mqtt.on_connect    = self._on_mqtt_connect
        self._mqtt.on_disconnect = self._on_mqtt_disconnect
        self._mqtt.on_message    = self._on_mqtt_message

        # Last-will: mark as disconnected if bridge crashes
        self._mqtt.will_set(TOPIC_CONNECTED, "false", retain=True)

    # ── MQTT ─────────────────────────────────────────────────────────────────

    def _mqtt_pub(self, topic: str, payload: str, retain: bool = False) -> None:
        self._mqtt.publish(topic, payload, retain=retain)

    def _on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        if rc == 0:
            log.info("MQTT connected to %s:%d", MQTT_BROKER, MQTT_PORT)
            client.subscribe([(TOPIC_POWER, 0), (TOPIC_JOYSTICK, 0)])
            log.info("MQTT subscribed: %s, %s", TOPIC_POWER, TOPIC_JOYSTICK)
        else:
            log.error("MQTT connection refused: rc=%d", rc)

    def _on_mqtt_disconnect(self, client, userdata, rc) -> None:
        log.warning("MQTT disconnected: rc=%d", rc)

    def _on_mqtt_message(self, client, userdata, msg) -> None:
        """
        Called from the MQTT thread.
        Parses the command and puts it into the asyncio cmd_queue.
        """
        topic   = msg.topic
        payload = msg.payload.decode("utf-8", errors="replace").strip()
        log.info("MQTT ← [%s] %r", topic, payload)

        cmd = None

        # ── vr2/power  →  critical queue (guaranteed delivery) ───────────────
        if topic == TOPIC_POWER:
            p = payload.lower()
            if p == "on":
                cmd = cmd_start()
                log.info("CMD: START")
            elif p == "off":
                cmd = cmd_shutdown()
                log.info("CMD: SHUTDOWN")
            else:
                log.warning("vr2/power: unknown payload %r (expected 'on'/'off')", payload)

            if cmd is not None and self._loop is not None:
                self._loop.call_soon_threadsafe(self._cmd_queue.put_nowait, cmd)

        # ── vr2/set_joystick  →  latest-wins slot (no queue buildup) ─────────
        elif topic == TOPIC_JOYSTICK:
            try:
                data = json.loads(payload)
                x = int(data.get("x", 0))
                y = int(data.get("y", 0))
                frame = cmd_joystick(x, y)
                log.debug("JOY x=%d y=%d", x, y)
                # Overwrite the slot — older joystick frames are simply discarded
                if self._loop is not None:
                    self._loop.call_soon_threadsafe(self._set_joy_latest, frame)
            except (json.JSONDecodeError, ValueError, TypeError) as exc:
                log.warning("vr2/set_joystick: invalid payload %r — %s", payload, exc)

    # ── BLE ──────────────────────────────────────────────────────────────────

    def _on_telemetry(self, _sender, data: bytearray) -> None:
        """
        BLE notify callback — called from asyncio loop.
        Parses telemetry and publishes changed values to MQTT.
        """
        parsed = parse_telemetry(data)
        if parsed is None:
            return

        state_name = parsed["state_name"]
        speed_str  = str(parsed["speed"])
        sw_json    = json.dumps(parse_switches(parsed["switches"]))

        if state_name != self._last_state:
            self._last_state = state_name
            self._mqtt_pub(TOPIC_STATE, state_name)
            log.info("BLE → MQTT  state: %s", state_name)

        if speed_str != self._last_speed:
            self._last_speed = speed_str
            self._mqtt_pub(TOPIC_SPEED, speed_str)
            log.info("BLE → MQTT  speed: %s", speed_str)

        if sw_json != self._last_switch:
            self._last_switch = sw_json
            self._mqtt_pub(TOPIC_SWITCH, sw_json)
            log.debug("BLE → MQTT  switch: %s", sw_json)

    def _set_joy_latest(self, frame: bytes) -> None:
        """Store the latest joystick frame (runs in asyncio thread — no lock needed)."""
        self._joy_latest = frame

    def _on_ble_disconnect(self, _client: BleakClient) -> None:
        log.warning("BLE disconnected (callback)")
        self._ble_connected = False

    async def _scan(self) -> Optional[object]:
        """Scan for the ESP32-S3 by device name or service UUID."""
        log.info("BLE scan — looking for '%s'  (timeout=%.0fs)", BLE_DEVICE_NAME, BLE_SCAN_TIMEOUT)

        def _match(device, adv_data):
            name_match = device.name and BLE_DEVICE_NAME in device.name
            uuid_match = any(
                SERVICE_UUID.lower() == str(u).lower()
                for u in (adv_data.service_uuids or [])
            )
            return name_match or uuid_match

        device = await BleakScanner.find_device_by_filter(_match, timeout=BLE_SCAN_TIMEOUT)

        if device:
            log.info("Found: %s [%s]", device.name, device.address)
        else:
            log.warning("Device '%s' not found", BLE_DEVICE_NAME)

        return device

    async def _ble_send(self, data: bytes) -> None:
        """Write a command frame to the ESP32 GATT characteristic."""
        if self._ble_client and self._ble_connected:
            try:
                await self._ble_client.write_gatt_char(
                    CHAR_WRITE_UUID, data, response=True
                )
            except Exception as exc:
                log.error("BLE write error: %s", exc)
        else:
            log.warning("BLE not connected — command dropped")

    async def _cmd_sender_task(self) -> None:
        """
        BLE write dispatcher — runs at 20 Hz (50 ms tick).

        Priority each tick:
          1. Critical commands (_cmd_queue) — power, speed: sent first, never dropped
          2. Joystick latest-wins slot     — only the most recent frame is sent;
             if a newer one arrived during the BLE write, the old one is already gone

        At 20 Hz the joystick update rate matches the Android app.
        Publishing faster than 20 Hz from MQTT is safe: extra frames are silently
        overwritten in _joy_latest and never accumulate.
        """
        TICK = 0.05   # 50 ms → 20 Hz

        while self._running:
            await asyncio.sleep(TICK)

            # ── 1. critical commands first ────────────────────────────────────
            while not self._cmd_queue.empty():
                try:
                    cmd = self._cmd_queue.get_nowait()
                    await self._ble_send(cmd)
                except asyncio.QueueEmpty:
                    break

            # ── 2. latest joystick frame (if any) ────────────────────────────
            if self._joy_latest is not None:
                frame, self._joy_latest = self._joy_latest, None
                await self._ble_send(frame)

    def _drain_cmd_queue(self) -> None:
        """Discard all pending commands accumulated while BLE was disconnected."""
        dropped = 0
        while not self._cmd_queue.empty():
            try:
                self._cmd_queue.get_nowait()
                dropped += 1
            except asyncio.QueueEmpty:
                break
        if self._joy_latest is not None:
            self._joy_latest = None
            dropped += 1
        if dropped:
            log.warning("Dropped %d stale command(s) (BLE was offline)", dropped)

    async def _ble_loop(self) -> None:
        """
        Main BLE connect / reconnect loop.
        Runs forever until self._running is False.

        Behaviour when device is not found:
          - _scan() returns None after BLE_SCAN_TIMEOUT seconds
          - waits BLE_RECONNECT_DELAY seconds, then retries — forever
          - MQTT bridge stays alive and responsive the whole time
          - vr2/connected remains "false" (retained)
          - incoming MQTT commands are queued but flushed on reconnect
            to avoid firing stale commands

        Behaviour if Bluetooth adapter is unavailable (exception in _scan):
          - caught here, logged as error
          - waits BLE_RECONNECT_DELAY and retries — bridge does NOT crash
        """
        while self._running:
            # ── scan (catches adapter errors so the bridge never crashes) ─────
            try:
                device = await self._scan()
            except Exception as exc:
                log.error("BLE scan error (adapter unavailable?): %s", exc)
                log.info("Retrying in %.0fs...", BLE_RECONNECT_DELAY)
                await asyncio.sleep(BLE_RECONNECT_DELAY)
                continue

            if device is None:
                log.info("Device not found — retrying scan in %.0fs...", BLE_RECONNECT_DELAY)
                await asyncio.sleep(BLE_RECONNECT_DELAY)
                continue

            try:
                async with BleakClient(
                    device,
                    disconnected_callback=self._on_ble_disconnect,
                ) as client:
                    self._ble_client    = client
                    self._ble_connected = True
                    log.info("BLE connected: %s", device.address)

                    # Drop commands that arrived while BLE was offline
                    self._drain_cmd_queue()

                    self._mqtt_pub(TOPIC_CONNECTED, "true", retain=True)

                    # Enable telemetry notifications
                    await client.start_notify(CHAR_TELEMETRY_UUID, self._on_telemetry)
                    log.info("Telemetry notifications enabled")

                    # Safety: send joystick center (0,0) immediately on connect
                    # Prevents any residual movement if the ESP32 had a stale joystick value
                    await client.write_gatt_char(CHAR_WRITE_UUID, cmd_joystick(0, 0), response=True)
                    log.info("Safety: joystick reset to (0, 0)")

                    # Hold connection until device disconnects or bridge stops
                    while self._running and client.is_connected:
                        await asyncio.sleep(0.5)

                    await client.stop_notify(CHAR_TELEMETRY_UUID)

            except Exception as exc:
                log.error("BLE error: %s", exc)

            finally:
                self._ble_client    = None
                self._ble_connected = False
                self._mqtt_pub(TOPIC_CONNECTED, "false", retain=True)

                # Reset cache so next connection re-publishes all values
                self._last_state  = None
                self._last_speed  = None
                self._last_switch = None

            if self._running:
                log.info("BLE disconnected — reconnecting in %.0fs...", BLE_RECONNECT_DELAY)
                await asyncio.sleep(BLE_RECONNECT_DELAY)

    # ── MAIN RUN ─────────────────────────────────────────────────────────────

    def _shutdown(self) -> None:
        log.info("Shutdown requested — stopping bridge...")
        self._running = False

    async def run(self) -> None:
        self._running = True
        self._loop    = asyncio.get_running_loop()

        # Register BlueZ pairing agent (for first-time BLE pairing)
        dbus_bus = await _register_bluez_agent(BLE_PIN)

        # Start MQTT network loop in background thread
        self._mqtt.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
        self._mqtt.loop_start()
        self._mqtt_pub(TOPIC_CONNECTED, "false", retain=True)
        log.info("MQTT loop started")

        # Graceful shutdown on SIGINT / SIGTERM
        for sig in (signal.SIGINT, signal.SIGTERM):
            self._loop.add_signal_handler(sig, self._shutdown)

        log.info("VR2 Bridge running — PIN=%d  MQTT=%s:%d",
                 BLE_PIN, MQTT_BROKER, MQTT_PORT)

        try:
            await asyncio.gather(
                self._ble_loop(),
                self._cmd_sender_task(),
            )
        finally:
            self._running = False
            self._mqtt_pub(TOPIC_CONNECTED, "false", retain=True)
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
            if dbus_bus:
                dbus_bus.disconnect()
            log.info("Bridge stopped.")


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    print(__doc__)
    bridge = VR2Bridge()
    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
