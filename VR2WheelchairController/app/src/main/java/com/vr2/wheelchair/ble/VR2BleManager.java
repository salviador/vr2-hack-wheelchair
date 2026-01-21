package com.vr2.wheelchair.ble;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.content.Context;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.lifecycle.LiveData;
import androidx.lifecycle.MutableLiveData;

import no.nordicsemi.android.ble.BleManager;
import no.nordicsemi.android.ble.data.Data;

/**
 * BLE Manager for VR2 PG Drives Wheelchair Controller Bridge
 * Uses Nordic BLE library for robust connection management
 *
 * Protocol with checksum:
 * - Commands: [cmd][data...][checksum] where checksum = (sum of previous bytes) XOR 0xFF
 * - Telemetry: [state][speed][battery][switches][checksum]
 */
public class VR2BleManager extends BleManager {

    private static final String TAG = "VR2BleManager";

    // Characteristics
    private BluetoothGattCharacteristic joyTxChar;      // Write commands
    private BluetoothGattCharacteristic telemetriaChar; // Receive notifications

    // Connection state
    private final MutableLiveData<Boolean> connected = new MutableLiveData<>(false);
    private final MutableLiveData<String> connectionStatus = new MutableLiveData<>("Disconnected");

    // Telemetry data
    private final MutableLiveData<TelemetryData> telemetryData = new MutableLiveData<>();

    // Callback interface
    public interface VR2Callback {
        void onConnected();
        void onDisconnected();
        void onTelemetryReceived(TelemetryData data);
        void onError(String message);
    }

    private VR2Callback callback;

    /**
     * Telemetry data class
     * Now includes switches byte and checksum validation
     */
    public static class TelemetryData {
        public final int state;
        public final int speed;
        public final int battery;
        public final int switches;      // 5 switches encoded in 1 byte
        public final boolean checksumValid;
        public final long timestamp;

        public TelemetryData(int state, int speed, int battery, int switches, boolean checksumValid) {
            this.state = state;
            this.speed = speed;
            this.battery = battery;
            this.switches = switches;
            this.checksumValid = checksumValid;
            this.timestamp = System.currentTimeMillis();
        }

        // Legacy constructor for backward compatibility
        public TelemetryData(int state, int speed, int battery) {
            this(state, speed, battery, 0, true);
        }

        public String getStateName() {
            return VR2BleConstants.getStateName(state);
        }

        public String getBatteryLevel() {
            return VR2BleConstants.getBatteryLevel(battery);
        }

        public boolean hasError() {
            return VR2BleConstants.isBatteryError(battery);
        }

        public String getErrorDescription() {
            return VR2BleConstants.getErrorDescription(battery);
        }

        // =====================================================================
        // Switch state methods (not displayed in UI but available for use)
        // =====================================================================

        /**
         * Check if sw2c (GPIO0) is pressed
         */
        public boolean isSw2cPressed() {
            return (switches & VR2BleConstants.SW_SW2C) != 0;
        }

        /**
         * Check if swl2 (GPIO45) is pressed
         */
        public boolean isSwl2Pressed() {
            return (switches & VR2BleConstants.SW_SWL2) != 0;
        }

        /**
         * Check if sw2r (GPIO35) is pressed
         */
        public boolean isSw2rPressed() {
            return (switches & VR2BleConstants.SW_SW2R) != 0;
        }

        /**
         * Check if sw2dw (GPIO37) is pressed
         */
        public boolean isSw2dwPressed() {
            return (switches & VR2BleConstants.SW_SW2DW) != 0;
        }

        /**
         * Check if sw2up (GPIO36) is pressed
         */
        public boolean isSw2upPressed() {
            return (switches & VR2BleConstants.SW_SW2UP) != 0;
        }

        /**
         * Check if any switch is pressed
         */
        public boolean isAnySwitchPressed() {
            return switches != 0;
        }

        /**
         * Get raw switches byte
         */
        public int getSwitchesRaw() {
            return switches;
        }
    }

    public VR2BleManager(@NonNull Context context) {
        super(context);
    }

    public void setCallback(VR2Callback callback) {
        this.callback = callback;
    }

    // =========================================================================
    // LiveData getters
    // =========================================================================

    public LiveData<Boolean> getConnectedState() {
        return connected;
    }

    public LiveData<String> getConnectionStatus() {
        return connectionStatus;
    }

    public LiveData<TelemetryData> getTelemetry() {
        return telemetryData;
    }

    // =========================================================================
    // BLE Manager implementation
    // =========================================================================

    @NonNull
    @Override
    protected BleManagerGattCallback getGattCallback() {
        return new VR2GattCallback();
    }

    /**
     * GATT callback implementation
     */
    private class VR2GattCallback extends BleManagerGattCallback {

        @Override
        protected boolean isRequiredServiceSupported(@NonNull BluetoothGatt gatt) {
            BluetoothGattService service = gatt.getService(VR2BleConstants.SERVICE_UUID);

            if (service == null) {
                Log.e(TAG, "VR2 service not found!");
                return false;
            }

            // Get joyTX characteristic (for writing commands)
            joyTxChar = service.getCharacteristic(VR2BleConstants.CHAR_JOY_TX_UUID);
            if (joyTxChar == null) {
                Log.e(TAG, "joyTX characteristic not found!");
                return false;
            }

            // Get telemetria characteristic (for notifications)
            telemetriaChar = service.getCharacteristic(VR2BleConstants.CHAR_TELEMETRIA_UUID);
            if (telemetriaChar == null) {
                Log.e(TAG, "Telemetria characteristic not found!");
                return false;
            }

            Log.i(TAG, "VR2 service and characteristics found");
            return true;
        }

        @Override
        protected void initialize() {
            Log.i(TAG, "Initializing VR2 connection...");

            // Enable notifications on telemetria characteristic
            setNotificationCallback(telemetriaChar)
                    .with((device, data) -> onTelemetryNotification(data));

            enableNotifications(telemetriaChar)
                    .done(device -> {
                        Log.i(TAG, "Telemetry notifications enabled");
                        connected.postValue(true);
                        connectionStatus.postValue("Connected");
                        if (callback != null) {
                            callback.onConnected();
                        }
                    })
                    .fail((device, status) -> {
                        Log.e(TAG, "Failed to enable notifications: " + status);
                        if (callback != null) {
                            callback.onError("Failed to enable notifications");
                        }
                    })
                    .enqueue();
        }

        @Override
        protected void onServicesInvalidated() {
            joyTxChar = null;
            telemetriaChar = null;
            connected.postValue(false);
            connectionStatus.postValue("Disconnected");
            if (callback != null) {
                callback.onDisconnected();
            }
        }
    }

    /**
     * Process telemetry notification data
     * New format: [state][speed][battery][switches][checksum] = 5 bytes
     * Legacy format: [state][speed][battery][reserved] = 4 bytes (no checksum)
     */
    private void onTelemetryNotification(Data data) {
        int size = data.size();

        if (size >= 5) {
            // New format with switches and checksum
            byte[] rawData = data.getValue();

            int state = data.getByte(0) & 0xFF;
            int speed = data.getByte(1) & 0xFF;
            int battery = data.getByte(2) & 0xFF;
            int switches = data.getByte(3) & 0xFF;

            // Verify checksum
            boolean checksumValid = VR2BleConstants.verifyChecksum(rawData);

            if (!checksumValid) {
                Log.w(TAG, String.format("Telemetry checksum INVALID: received=0x%02X",
                        rawData[4] & 0xFF));
            }

            TelemetryData telemetry = new TelemetryData(state, speed, battery, switches, checksumValid);
            telemetryData.postValue(telemetry);

            Log.d(TAG, String.format("Telemetry: state=%s speed=%d battery=0x%02X sw=0x%02X chk=%s",
                    telemetry.getStateName(), speed, battery, switches,
                    checksumValid ? "OK" : "FAIL"));

            if (callback != null) {
                callback.onTelemetryReceived(telemetry);
            }

        } else if (size >= 3) {
            // Legacy format (backward compatibility)
            int state = data.getByte(0) & 0xFF;
            int speed = data.getByte(1) & 0xFF;
            int battery = data.getByte(2) & 0xFF;

            TelemetryData telemetry = new TelemetryData(state, speed, battery, 0, true);
            telemetryData.postValue(telemetry);

            Log.d(TAG, String.format("Telemetry (legacy): state=%s speed=%d battery=0x%02X",
                    telemetry.getStateName(), speed, battery));

            if (callback != null) {
                callback.onTelemetryReceived(telemetry);
            }
        } else {
            Log.w(TAG, "Telemetry data too short: " + size + " bytes");
        }
    }

    // =========================================================================
    // Command methods (with checksum)
    // =========================================================================

    /**
     * Send joystick values with checksum
     * Format: [0x01][X][Y][buttons][checksum]
     * @param x X axis value (-127 to +127)
     * @param y Y axis value (-127 to +127)
     * @param buttons Button flags
     */
    public void sendJoystick(int x, int y, int buttons) {
        if (joyTxChar == null) {
            Log.w(TAG, "Not connected - cannot send joystick");
            return;
        }

        // Clamp values
        final int clampedX = Math.max(-127, Math.min(127, x));
        final int clampedY = Math.max(-127, Math.min(127, y));
        final int btn = buttons & 0xFF;

        // Build data without checksum
        byte[] dataWithoutChecksum = new byte[] {
                VR2BleConstants.CMD_SET_JOY,
                (byte) clampedX,
                (byte) clampedY,
                (byte) btn
        };

        // Append checksum
        byte[] data = VR2BleConstants.appendChecksum(dataWithoutChecksum);

        writeCharacteristic(joyTxChar, data, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
                .done(device -> Log.v(TAG, String.format("JOY sent: x=%d y=%d btn=0x%02X chk=0x%02X",
                        clampedX, clampedY, btn, data[4] & 0xFF)))
                .fail((device, status) -> Log.e(TAG, "Failed to send JOY: " + status))
                .enqueue();
    }

    /**
     * Send simple joystick values (no buttons)
     */
    public void sendJoystick(int x, int y) {
        sendJoystick(x, y, VR2BleConstants.BTN_NONE);
    }

    /**
     * Send START command to activate ECU
     * Format: [0x02][checksum]
     */
    public void sendStart() {
        sendSimpleCommand(VR2BleConstants.CMD_START, "START");
    }

    /**
     * Send SHUTDOWN command
     * Format: [0x03][checksum]
     */
    public void sendShutdown() {
        sendSimpleCommand(VR2BleConstants.CMD_SHUTDOWN, "SHUTDOWN");
    }

    /**
     * Send SPEED+ command
     * Format: [0x04][checksum]
     */
    public void sendSpeedPlus() {
        sendSimpleCommand(VR2BleConstants.CMD_SPEED_PLUS, "SPEED+");
    }

    /**
     * Send SPEED- command
     * Format: [0x05][checksum]
     */
    public void sendSpeedMinus() {
        sendSimpleCommand(VR2BleConstants.CMD_SPEED_MINUS, "SPEED-");
    }

    /**
     * Send single-byte command with checksum
     * Format: [cmd][checksum]
     */
    private void sendSimpleCommand(byte cmd, String cmdName) {
        if (joyTxChar == null) {
            Log.w(TAG, "Not connected - cannot send " + cmdName);
            return;
        }

        // Build data with checksum
        byte[] dataWithoutChecksum = new byte[] { cmd };
        byte[] data = VR2BleConstants.appendChecksum(dataWithoutChecksum);

        writeCharacteristic(joyTxChar, data, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
                .done(device -> Log.i(TAG, String.format("%s sent: [0x%02X][0x%02X]",
                        cmdName, data[0] & 0xFF, data[1] & 0xFF)))
                .fail((device, status) -> Log.e(TAG, "Failed to send " + cmdName + ": " + status))
                .enqueue();
    }

    /**
     * Send emergency stop (center joystick)
     */
    public void sendStop() {
        sendJoystick(0, 0, VR2BleConstants.BTN_NONE);
    }

    // =========================================================================
    // Connection management
    // =========================================================================

    /**
     * Connect to VR2 device
     */
    public void connectToDevice(@NonNull BluetoothDevice device) {
        connectionStatus.postValue("Connecting...");

        connect(device)
                .retry(3, 200)
                .useAutoConnect(false)
                .timeout(10000)
                .done(d -> Log.i(TAG, "Connected to " + d.getName()))
                .fail((d, status) -> {
                    Log.e(TAG, "Connection failed: " + status);
                    connectionStatus.postValue("Connection failed");
                    if (callback != null) {
                        callback.onError("Connection failed: " + status);
                    }
                })
                .enqueue();
    }

    /**
     * Disconnect from device
     */
    public void disconnectDevice() {
        disconnect().enqueue();
    }
}