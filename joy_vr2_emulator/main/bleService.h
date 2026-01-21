/**
 * @file bleService.h
 * @brief BLE GATT Service for VR2 PG Drives Wheelchair Controller Bridge
 * 
 * BLE Protocol:
 * 
 * WRITE to Characteristic B (joyTX):
 *   CMD 0x01 - SET_JOY:     [0x01] [X:int8] [Y:int8] [buttons:uint8] [checksum:uint8]
 *   CMD 0x02 - START:       [0x02] [checksum:uint8]
 *   CMD 0x03 - SHUTDOWN:    [0x03] [checksum:uint8]
 *   CMD 0x04 - SPEED_PLUS:  [0x04] [checksum:uint8]
 *   CMD 0x05 - SPEED_MINUS: [0x05] [checksum:uint8]
 *   
 *   Checksum = (sum of all previous bytes) XOR 0xFF
 *   
 * NOTIFY from Characteristic TELEMETRIA:
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

#ifndef __BLESERVICE_H__
#define __BLESERVICE_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/message_buffer.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"

/*============================================================================
 * BLE COMMAND TYPES (from client)
 *============================================================================*/
#define BLE_CMD_SET_JOY      0x01  /* [cmd][x:int8][y:int8][buttons:uint8][checksum] */
#define BLE_CMD_START        0x02  /* [cmd][checksum] */
#define BLE_CMD_SHUTDOWN     0x03  /* [cmd][checksum] */
#define BLE_CMD_SPEED_PLUS   0x04  /* [cmd][checksum] */
#define BLE_CMD_SPEED_MINUS  0x05  /* [cmd][checksum] */

/*============================================================================
 * BLE TELEMETRY STATE VALUES (to client)
 *============================================================================*/
#define BLE_STATE_IDLE       0x00
#define BLE_STATE_INIT       0x01
#define BLE_STATE_RUN        0x02
#define BLE_STATE_ERROR      0x03
#define BLE_STATE_SHUTDOWN   0x04

/*============================================================================
 * BLE BUTTON FLAGS (in SET_JOY command)
 *============================================================================*/
#define BLE_BTN_NONE         0x00
#define BLE_BTN_BRAKE        0x01  /* z button */
#define BLE_BTN_BOOST        0x02  /* c button */
#define BLE_BTN_SPEED_MINUS  0x04
#define BLE_BTN_SPEED_PLUS   0x08

/*============================================================================
 * TELEMETRY SWITCHES ENCODING (5 buttons)
 *============================================================================*/
#define TELE_SW_SW2C         0x01  /* GPIO0  - bit 0 */
#define TELE_SW_SWL2         0x02  /* GPIO45 - bit 1 */
#define TELE_SW_SW2R         0x04  /* GPIO35 - bit 2 */
#define TELE_SW_SW2DW        0x08  /* GPIO37 - bit 3 */
#define TELE_SW_SW2UP        0x10  /* GPIO36 - bit 4 */

/*============================================================================
 * EVENT GROUP BITS
 *============================================================================*/
#define BIT_0               (1 << 0)
#define BIT_1               (1 << 1)
#define BIT_2               (1 << 2)

#define BLECONNECTED_BIT            BIT_0
#define BLECONNECTED_BIT_JOYSTICK   BIT_1

extern EventGroupHandle_t BLE_event_group;
extern MessageBufferHandle_t BLE_SendNotification_Queue;

/*============================================================================
 * GATT SERVICE ATTRIBUTE TABLE INDICES
 *============================================================================*/
enum {
    IDX_SVC,
    
    /* Characteristic A - RX from ESP to Client (Notify) */
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    /* Characteristic B - TX from Client to ESP (Write) */
    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    /* Characteristic TELEMETRIA - ECU State Notify */
    IDX_CHAR_TELEMETRIA,
    IDX_CHAR_VAL_TELEMETRIA,
    IDX_CHAR_CFG_TELEMETRIA,

    HRS_IDX_NB,
};

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

/**
 * @brief Legacy joystick data structure (7 bytes packed)
 * Used for backwards compatibility with existing clients
 */
struct CHART_data_TX {
    int16_t uJoy_x;       /* X axis: -32768 to +32767 (scaled to -127..+127) */
    int16_t uJoy_y;       /* Y axis: -32768 to +32767 (scaled to -127..+127) */
    int16_t uvbattery;    /* Battery voltage (not used in bridge) */
    uint8_t buttons;      /* Button flags */
} __attribute__((packed));

/**
 * @brief Telemetry data structure (5 bytes)
 * Sent via notification to client
 */
struct BLE_Telemetry_TX {
    uint8_t state;        /* ECU state: IDLE/INIT/RUN/ERROR/SHUTDOWN */
    uint8_t speed;        /* Current speed setting (1-5) */
    uint8_t battery;      /* Battery level code */
    uint8_t switches;     /* 5 switches encoded: sw2c|swl2|sw2r|sw2dw|sw2up */
    uint8_t checksum;     /* (sum bytes 0-3) XOR 0xFF */
} __attribute__((packed));

/**
 * @brief Battery status data structure (legacy)
 */
struct JOYSTICK_BATTERY_DATA {
    uint8_t type_message;
    int16_t Battery;
} __attribute__((packed));

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Start BLE GATT service
 * Initializes NVS, Bluetooth controller, and GATT server.
 * Creates telemetry task for VR2 -> BLE communication.
 */
void BLE_Service_Start(void);

/**
 * @brief Stop BLE GATT service
 * Disconnects clients, stops advertising, and deinitializes Bluetooth stack.
 */
void BLE_Service_Stop(void);

/**
 * @brief Check if a BLE client is connected
 * @return true if connected, false otherwise
 */
bool BLE_Is_Connected(void);

/**
 * @brief Task to send BLE notifications
 * @param pvParameter Task parameter (unused)
 */
void Send_BLE_Notification_Task(void *pvParameter);

/**
 * @brief Handle client connection
 * @param gatts_if GATT interface
 * @param param Connection parameters
 */
void connected(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/**
 * @brief Handle client disconnection
 * @param param Disconnection parameters
 */
void disconnected(esp_ble_gatts_cb_param_t *param);

/**
 * @brief Calculate checksum for BLE data
 * @param data Pointer to data bytes
 * @param len Number of bytes
 * @return Checksum = (sum of bytes) XOR 0xFF
 */
uint8_t ble_calc_checksum(const uint8_t *data, size_t len);

/**
 * @brief Verify checksum of received BLE data
 * @param data Pointer to data bytes (including checksum as last byte)
 * @param len Total length including checksum
 * @return true if checksum valid, false otherwise
 */
bool ble_verify_checksum(const uint8_t *data, size_t len);

#endif /* __BLESERVICE_H__ */