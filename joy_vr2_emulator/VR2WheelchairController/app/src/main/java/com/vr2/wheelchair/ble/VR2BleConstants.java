package com.vr2.wheelchair.ble;

import java.util.UUID;

/**
 * BLE UUIDs and command constants for VR2 PG Drives Wheelchair Controller Bridge
 *
 * Protocol (from ESP32 firmware):
 *
 * WRITE to joyTX characteristic (with checksum):
 *   CMD 0x01 - SET_JOY:     [0x01] [X:int8] [Y:int8] [buttons:uint8] [checksum]
 *   CMD 0x02 - START:       [0x02] [checksum]
 *   CMD 0x03 - SHUTDOWN:    [0x03] [checksum]
 *   CMD 0x04 - SPEED_PLUS:  [0x04] [checksum]
 *   CMD 0x05 - SPEED_MINUS: [0x05] [checksum]
 *
 *   Checksum = (sum of all previous bytes) XOR 0xFF
 *
 * NOTIFY from TELEMETRIA characteristic:
 *   Format: [state:uint8] [speed:uint8] [battery:uint8] [switches:uint8] [checksum:uint8]
 *
 *   Switches byte encoding (active HIGH = pressed):
 *     bit 0 (0x01): sw2c  (GPIO0)
 *     bit 1 (0x02): swl2  (GPIO45)
 *     bit 2 (0x04): sw2r  (GPIO35)
 *     bit 3 (0x08): sw2dw (GPIO37)
 *     bit 4 (0x10): sw2up (GPIO36)
 *
 *   Checksum = (sum of bytes 0-3) XOR 0xFF
 */
public class VR2BleConstants {

    // Device name advertised by ESP32
    public static final String DEVICE_NAME = "WHEELS Mic";

    // Service UUID (from bleService.c)
    // In firmware: 0x3d,0x23,0x33,0xa0,0xde,0xf9,0x42,0x88,0x30,0x32,0x12,0xd7,0x11,0x37,0x34,0xda
    // Converted to standard UUID format (little endian to big endian)
    public static final UUID SERVICE_UUID =
            UUID.fromString("da343711-d712-3230-8842-f9dea033233d");

    // Characteristic joyTX - Client writes commands here (0xf8 in byte 5)
    public static final UUID CHAR_JOY_TX_UUID =
            UUID.fromString("da343711-d712-3230-8842-f7dea033233d");

    // Characteristic joyRX - ESP to Client notifications (0xf7 in byte 5) - legacy
    public static final UUID CHAR_JOY_RX_UUID =
            UUID.fromString("da343711-d712-3230-8842-f8dea033233d");

    // Characteristic TELEMETRIA - ECU state notifications (0xf6 in byte 5)
    public static final UUID CHAR_TELEMETRIA_UUID =
            UUID.fromString("da343711-d712-3230-8842-f6dea033233d");

    // Client Characteristic Configuration Descriptor (for notifications)
    public static final UUID CCCD_UUID =
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    // =========================================================================
    // BLE Commands (write to CHAR_JOY_TX)
    // =========================================================================

    public static final byte CMD_SET_JOY      = 0x01;  // [cmd][x][y][buttons][checksum]
    public static final byte CMD_START        = 0x02;  // [cmd][checksum]
    public static final byte CMD_SHUTDOWN     = 0x03;  // [cmd][checksum]
    public static final byte CMD_SPEED_PLUS   = 0x04;  // [cmd][checksum]
    public static final byte CMD_SPEED_MINUS  = 0x05;  // [cmd][checksum]

    // =========================================================================
    // Button flags (in SET_JOY command)
    // =========================================================================

    public static final byte BTN_NONE        = 0x00;
    public static final byte BTN_BRAKE       = 0x01;  // z button
    public static final byte BTN_BOOST       = 0x02;  // c button
    public static final byte BTN_SPEED_MINUS = 0x04;
    public static final byte BTN_SPEED_PLUS  = 0x08;

    // =========================================================================
    // Telemetry switch flags (from TELEMETRIA notifications - byte 3)
    // =========================================================================

    public static final int SW_SW2C  = 0x01;  // GPIO0  - bit 0
    public static final int SW_SWL2  = 0x02;  // GPIO45 - bit 1
    public static final int SW_SW2R  = 0x04;  // GPIO35 - bit 2
    public static final int SW_SW2DW = 0x08;  // GPIO37 - bit 3
    public static final int SW_SW2UP = 0x10;  // GPIO36 - bit 4

    // =========================================================================
    // Telemetry state values (from TELEMETRIA notifications)
    // =========================================================================

    public static final int STATE_IDLE     = 0x00;
    public static final int STATE_INIT     = 0x01;
    public static final int STATE_RUN      = 0x02;
    public static final int STATE_ERROR    = 0x03;
    public static final int STATE_SHUTDOWN = 0x04;

    // =========================================================================
    // Checksum calculation
    // =========================================================================

    /**
     * Calculate checksum for BLE data
     * @param data Data bytes (without checksum)
     * @return Checksum = (sum of bytes) XOR 0xFF
     */
    public static byte calculateChecksum(byte[] data) {
        int sum = 0;
        for (byte b : data) {
            sum += (b & 0xFF);
        }
        return (byte) ((sum & 0xFF) ^ 0xFF);
    }

    /**
     * Calculate checksum for partial data
     * @param data Data bytes
     * @param length Number of bytes to include
     * @return Checksum = (sum of bytes) XOR 0xFF
     */
    public static byte calculateChecksum(byte[] data, int length) {
        int sum = 0;
        for (int i = 0; i < length; i++) {
            sum += (data[i] & 0xFF);
        }
        return (byte) ((sum & 0xFF) ^ 0xFF);
    }

    /**
     * Verify checksum of received data
     * @param data Data including checksum as last byte
     * @return true if checksum valid
     */
    public static boolean verifyChecksum(byte[] data) {
        if (data == null || data.length < 2) return false;

        byte expected = calculateChecksum(data, data.length - 1);
        byte received = data[data.length - 1];

        return expected == received;
    }

    /**
     * Append checksum to data array
     * @param dataWithoutChecksum Original data
     * @return New array with checksum appended
     */
    public static byte[] appendChecksum(byte[] dataWithoutChecksum) {
        byte[] result = new byte[dataWithoutChecksum.length + 1];
        System.arraycopy(dataWithoutChecksum, 0, result, 0, dataWithoutChecksum.length);
        result[result.length - 1] = calculateChecksum(dataWithoutChecksum);
        return result;
    }

    // =========================================================================
    // Helper methods
    // =========================================================================

    /**
     * Get state name string
     */
    public static String getStateName(int state) {
        switch (state) {
            case STATE_IDLE:     return "IDLE";
            case STATE_INIT:     return "INIT";
            case STATE_RUN:      return "RUN";
            case STATE_ERROR:    return "ERROR";
            case STATE_SHUTDOWN: return "SHUTDOWN";
            default:             return "UNKNOWN";
        }
    }

    /**
     * Get battery level description from ECU battery code
     */
    public static String getBatteryLevel(int batteryCode) {
        switch (batteryCode & 0xF0) {
            case 0x10: return "1 LED (Low)";
            case 0x20: return "2 LED";
            case 0x30: return "3 LED (Red)";
            case 0x40: return "4 LED (Orange)";
            case 0x50: return "5 LED";
            case 0x60: return "6 LED";
            case 0x70: return "7 LED";
            case 0x80: return "8 LED (Green)";
            case 0x90: return "9 LED (Green)";
            case 0xA0: return "10 LED (Full)";
            default:   return String.format("0x%02X", batteryCode);
        }
    }

    /**
     * Check if battery code indicates an error
     */
    public static boolean isBatteryError(int batteryCode) {
        return batteryCode == 0x23 || batteryCode == 0x43 || batteryCode == 0x93;
    }

    /**
     * Get error description from battery code
     */
    public static String getErrorDescription(int batteryCode) {
        switch (batteryCode) {
            case 0x23: return "Motor Left Error";
            case 0x43: return "Motor Right Error";
            case 0x93: return "Brake Error";
            default:   return null;
        }
    }

    /**
     * Check if a specific switch is pressed
     * @param switches The switches byte from telemetry
     * @param switchMask The switch mask (SW_SW2C, SW_SWL2, etc.)
     * @return true if switch is pressed
     */
    public static boolean isSwitchPressed(int switches, int switchMask) {
        return (switches & switchMask) != 0;
    }
}