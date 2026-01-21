/*
 * BLE Service for VR2 PG Drives Wheelchair Controller Bridge
 * 
 * BLE Protocol:
 * 
 * CHARACTERISTIC B (joyTX) - Client writes commands here:
 *   Format: [CMD_TYPE] [DATA...] [CHECKSUM]
 *   
 *   CMD 0x01 - SET_JOY:     [0x01] [X:int8] [Y:int8] [buttons:uint8] [checksum]
 *   CMD 0x02 - START:       [0x02] [checksum]
 *   CMD 0x03 - SHUTDOWN:    [0x03] [checksum]
 *   CMD 0x04 - SPEED_PLUS:  [0x04] [checksum]
 *   CMD 0x05 - SPEED_MINUS: [0x05] [checksum]
 *   
 *   Checksum = (sum of all previous bytes) XOR 0xFF
 *   
 * CHARACTERISTIC TELEMETRIA - Server notifies ECU state:
 *   Format: [state:uint8] [speed:uint8] [battery:uint8] [switches:uint8] [checksum:uint8]
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "bleService.h"
#include "vr2_pgdrives.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/message_buffer.h"
#include "driver/gpio.h"


//#define FILTER_MAC  1
//#define MULTICONNECTION
//#define BLEPASSORD
#define EXTENDED_ADVERSING_5

/* BLE Command Types from client */
#define BLE_CMD_SET_JOY      0x01
#define BLE_CMD_START        0x02
#define BLE_CMD_SHUTDOWN     0x03
#define BLE_CMD_SPEED_PLUS   0x04
#define BLE_CMD_SPEED_MINUS  0x05

static const char *TAG_BLE = "BLE_VR2";

/* Global variables - defined here, declared extern in header */
EventGroupHandle_t BLE_event_group = NULL;
MessageBufferHandle_t BLE_SendNotification_Queue = NULL;

struct _clientsConnection
{
    uint8_t smartphone_en;
    uint8_t smartphone_connid;
    
    uint8_t joy_en;
    uint8_t joy_connid;

    uint8_t login_connid;
    uint8_t numero_connessioni;
};
static struct _clientsConnection clientsConnection;

/* BLECONNECTED_BIT and BLECONNECTED_BIT_JOYSTICK are defined in bleService.h */

const uint8_t MAC_REMOTO_JOYSTICK[6] =   {  0x02, 0x80, 0xe1, 0x00, 0x00, 0xe0 };

#define GATTS_TABLE_TAG "SEC_GATTS_DEMO"

#define JOYSTICK_PROFILE_NUM                      1
#define JOYSTICK_PROFILE_APP_IDX                  0
#define ESP_JOYSTICK_APP_ID                      0x55
#define HEART_RATE_SVC_INST_ID                    0
#define EXT_ADV_HANDLE                            0
#define NUM_EXT_ADV_SET                           1
#define EXT_ADV_DURATION                          0
#define EXT_ADV_MAX_EVENTS                        0

#define GATTS_DEMO_CHAR_VAL_LEN_MAX               0x40

static uint16_t BLE_wheels_handle_table[HRS_IDX_NB];

struct CHART_data_TX ble_data_receiver;
struct CHART_data_TX *pble_data_receiver;

const static char DEVICE_NAME[] = {'W', 'H', 'E', 'E', 'L', 'S', ' ', 'M', 'i', 'c'};

/* Service UUID */
const uint8_t service_uuid[16] = {
    0x3d, 0x23, 0x33, 0xa0, 0xde, 0xf9, 0x42, 0x88, 
    0x30, 0x32, 0x12, 0xd7, 0x11, 0x37, 0x34, 0xda
};

/* Characteristic UUIDs */
static uint8_t gatt_char_joyTX_uuid[16] = {
    0x3d, 0x23, 0x33, 0xa0, 0xde, 0xf8, 0x42, 0x88, 
    0x30, 0x32, 0x12, 0xd7, 0x11, 0x37, 0x34, 0xda
};
static uint8_t gatt_char_joyRX_uuid[16] = {
    0x3d, 0x23, 0x33, 0xa0, 0xde, 0xf7, 0x42, 0x88, 
    0x30, 0x32, 0x12, 0xd7, 0x11, 0x37, 0x34, 0xda
};
static uint8_t gatt_char_TELEMETRIA_uuid[16] = {
    0x3d, 0x23, 0x33, 0xa0, 0xde, 0xf6, 0x42, 0x88, 
    0x30, 0x32, 0x12, 0xd7, 0x11, 0x37, 0x34, 0xda
};

#ifdef EXTENDED_ADVERSING_5
static uint8_t ext_adv_raw_data[] = {
    0x02, 0x01, 0x06,
    0x11, 0x07, 
    service_uuid[0], service_uuid[1], service_uuid[2], service_uuid[3], 
    service_uuid[4], service_uuid[5], service_uuid[6], service_uuid[7], 
    service_uuid[8], service_uuid[9], service_uuid[10], service_uuid[11], 
    service_uuid[12], service_uuid[13], service_uuid[14], service_uuid[15],
    0x0B, 0X09, 'W', 'H', 'E', 'E', 'L', 'S', ' ', 'M', 'i', 'c',
};

static esp_ble_gap_ext_adv_t ext_adv[1] = {
    [0] = {EXT_ADV_HANDLE, EXT_ADV_DURATION, EXT_ADV_MAX_EVENTS},
};

esp_ble_gap_ext_adv_params_t ext_adv_params_2M = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
    .interval_min = 0x20,
    .interval_max = 0x20,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_2M,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
};
#else
static esp_ble_gap_ext_adv_t ext_adv[1] = {
    [0] = {EXT_ADV_HANDLE, EXT_ADV_DURATION, EXT_ADV_MAX_EVENTS},
};

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t raw_adv_data[] = {
    0x02, 0x01, 0x06,
    0x11, 0x07, 
    service_uuid[0], service_uuid[1], service_uuid[2], service_uuid[3], 
    service_uuid[4], service_uuid[5], service_uuid[6], service_uuid[7], 
    service_uuid[8], service_uuid[9], service_uuid[10], service_uuid[11], 
    service_uuid[12], service_uuid[13], service_uuid[14], service_uuid[15],
    0x06, 0X09, 'W', 'H', 'E', 'E', 'L',
};

static uint8_t raw_scan_rsp_data[] = {  
    0x02, 0x01, 0x06,
    0x02, 0x0a, 0xeb,
    0x11, 0x07, 
    service_uuid[0], service_uuid[1], service_uuid[2], service_uuid[3], 
    service_uuid[4], service_uuid[5], service_uuid[6], service_uuid[7], 
    service_uuid[8], service_uuid[9], service_uuid[10], service_uuid[11], 
    service_uuid[12], service_uuid[13], service_uuid[14], service_uuid[15],
    0x0B, 0X09, 'W', 'H', 'E', 'E', 'L', 'S', ' ', 'M', 'i', 'c',
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
#endif

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst Joystick_profile_tab[JOYSTICK_PROFILE_NUM] = {
    [JOYSTICK_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify2  = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

static const uint8_t char_notification_ccc[2]      = {0x00, 0x00};
static const uint8_t char_value_charRX[]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t char_value_charTX[]           = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t char_value_TELEMETRIA[]       = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))
#define SVC_INST_ID                 0

/* GATT Database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    [IDX_SVC] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(service_uuid), sizeof(service_uuid), (uint8_t *)&service_uuid}},

    /* Characteristic A - RX from ESP to Client (Notify) */
    [IDX_CHAR_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&gatt_char_joyRX_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_charRX), (uint8_t *)char_value_charRX}},

    [IDX_CHAR_CFG_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(char_notification_ccc), (uint8_t *)char_notification_ccc}},

    /* Characteristic B - TX from Client to ESP (Write) */
    [IDX_CHAR_B] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

    [IDX_CHAR_VAL_B] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&gatt_char_joyTX_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_charTX), (uint8_t *)char_value_charTX}},

    /* Characteristic TELEMETRIA - ECU State Notify */
    [IDX_CHAR_TELEMETRIA] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify2}},

    [IDX_CHAR_VAL_TELEMETRIA] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_128, (uint8_t *)&gatt_char_TELEMETRIA_uuid, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value_TELEMETRIA), (uint8_t *)char_value_TELEMETRIA}},

    [IDX_CHAR_CFG_TELEMETRIA] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(char_notification_ccc), (uint8_t *)char_notification_ccc}},
};

/*============================================================================
 * CHECKSUM FUNCTIONS
 *============================================================================*/

/**
 * @brief Calculate checksum: (sum of bytes) XOR 0xFF
 */
uint8_t ble_calc_checksum(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum ^ 0xFF;
}

/**
 * @brief Verify checksum of received data
 * @param data Data including checksum as last byte
 * @param len Total length including checksum
 * @return true if valid
 */
bool ble_verify_checksum(const uint8_t *data, size_t len)
{
    if (len < 2) return false;
    
    uint8_t expected = ble_calc_checksum(data, len - 1);
    uint8_t received = data[len - 1];
    
    if (expected != received) {
        ESP_LOGW(TAG_BLE, "Checksum mismatch: expected 0x%02X, got 0x%02X", expected, received);
        return false;
    }
    return true;
}

/*============================================================================
 * 5-BUTTON DEBOUNCE SYSTEM
 *============================================================================*/

/* GPIO pins for the 5 telemetry switches */
#define TELE_SW_SW2C_PIN    GPIO_NUM_0    /* bit 0 = 0x01 */
#define TELE_SW_SWL2_PIN    GPIO_NUM_45   /* bit 1 = 0x02 */
#define TELE_SW_SW2R_PIN    GPIO_NUM_35   /* bit 2 = 0x04 */
#define TELE_SW_SW2DW_PIN   GPIO_NUM_37   /* bit 3 = 0x08 */
#define TELE_SW_SW2UP_PIN   GPIO_NUM_36   /* bit 4 = 0x10 */

#define TELE_SW_NUM_BUTTONS 5
#define TELE_SW_DEBOUNCE_MS 30  /* Debounce time in ms */

/* Debounce state for each button */
typedef struct {
    gpio_num_t pin;
    uint8_t mask;
    bool last_raw;
    bool debounced;
    uint32_t last_change_tick;
} tele_sw_debounce_t;

static tele_sw_debounce_t tele_sw_buttons[TELE_SW_NUM_BUTTONS] = {
    { TELE_SW_SW2C_PIN,  TELE_SW_SW2C,  false, false, 0 },
    { TELE_SW_SWL2_PIN,  TELE_SW_SWL2,  false, false, 0 },
    { TELE_SW_SW2R_PIN,  TELE_SW_SW2R,  false, false, 0 },
    { TELE_SW_SW2DW_PIN, TELE_SW_SW2DW, false, false, 0 },
    { TELE_SW_SW2UP_PIN, TELE_SW_SW2UP, false, false, 0 },
};

static bool tele_sw_initialized = false;

/**
 * @brief Initialize the 5 telemetry switch GPIOs
 */
static void tele_sw_init(void)
{
    if (tele_sw_initialized) return;
    
    ESP_LOGI(TAG_BLE, "Initializing 5 telemetry switch GPIOs with pull-up...");
    ESP_LOGW(TAG_BLE, "WARNING: GPIO0 and GPIO45 are strapping pins!");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TELE_SW_SW2C_PIN) |
                        (1ULL << TELE_SW_SWL2_PIN) |
                        (1ULL << TELE_SW_SW2R_PIN) |
                        (1ULL << TELE_SW_SW2DW_PIN) |
                        (1ULL << TELE_SW_SW2UP_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_BLE, "Failed to configure telemetry switch GPIOs: %s", esp_err_to_name(ret));
        return;
    }
    
    /* Initialize debounce state */
    uint32_t now = xTaskGetTickCount();
    for (int i = 0; i < TELE_SW_NUM_BUTTONS; i++) {
        tele_sw_buttons[i].last_raw = false;
        tele_sw_buttons[i].debounced = false;
        tele_sw_buttons[i].last_change_tick = now;
    }
    
    ESP_LOGI(TAG_BLE, "Telemetry switches initialized:");
    ESP_LOGI(TAG_BLE, "  sw2c  -> GPIO%d (0x01)", TELE_SW_SW2C_PIN);
    ESP_LOGI(TAG_BLE, "  swl2  -> GPIO%d (0x02)", TELE_SW_SWL2_PIN);
    ESP_LOGI(TAG_BLE, "  sw2r  -> GPIO%d (0x04)", TELE_SW_SW2R_PIN);
    ESP_LOGI(TAG_BLE, "  sw2dw -> GPIO%d (0x08)", TELE_SW_SW2DW_PIN);
    ESP_LOGI(TAG_BLE, "  sw2up -> GPIO%d (0x10)", TELE_SW_SW2UP_PIN);
    
    tele_sw_initialized = true;
}

/**
 * @brief Read all 5 switches with debounce
 * @return Byte with debounced switch states (1 = pressed)
 */
static uint8_t tele_sw_read_debounced(void)
{
    uint32_t now = xTaskGetTickCount();
    uint8_t result = 0;
    
    for (int i = 0; i < TELE_SW_NUM_BUTTONS; i++) {
        /* Read current raw state (active LOW: 0 = pressed) */
        bool raw_pressed = (gpio_get_level(tele_sw_buttons[i].pin) == 0);
        
        /* Check if state changed */
        if (raw_pressed != tele_sw_buttons[i].last_raw) {
            tele_sw_buttons[i].last_raw = raw_pressed;
            tele_sw_buttons[i].last_change_tick = now;
        }
        
        /* Apply debounce: only update debounced state if stable for DEBOUNCE_MS */
        uint32_t elapsed_ms = (now - tele_sw_buttons[i].last_change_tick) * portTICK_PERIOD_MS;
        if (elapsed_ms >= TELE_SW_DEBOUNCE_MS) {
            tele_sw_buttons[i].debounced = tele_sw_buttons[i].last_raw;
        }
        
        /* Build result byte */
        if (tele_sw_buttons[i].debounced) {
            result |= tele_sw_buttons[i].mask;
        }
    }
    
    return result;
}

/*============================================================================
 * BLE -> VR2 BRIDGE FUNCTIONS
 *============================================================================*/

/**
 * Process BLE command and forward to VR2
 * Now includes checksum verification
 */
static void ble_process_command(const uint8_t *data, size_t len)
{
    if (len < 2) {
        ESP_LOGW(TAG_BLE, "Command too short (need at least cmd + checksum)");
        return;
    }
    
    /* Verify checksum */
    if (!ble_verify_checksum(data, len)) {
        ESP_LOGE(TAG_BLE, "Command rejected: invalid checksum");
        return;
    }
    
    vr2_command_t cmd = {0};
    uint8_t cmd_type = data[0];
    
    switch (cmd_type) {
        case BLE_CMD_SET_JOY:
            /* Format: [0x01][X][Y][buttons][checksum] = 5 bytes */
            if (len >= 5) {
                cmd.cmd = VR2_CMD_SET_JOY;
                cmd.x = (int8_t)data[1];
                cmd.y = (int8_t)data[2];
                if(cmd.x > 100){
                    cmd.x = 100;
                }
                if(cmd.x < -100){
                    cmd.x = -100;
                }
                if(cmd.y > 100){
                    cmd.y = 100;
                }
                if(cmd.y < -100){
                    cmd.y = -100;
                }            
                cmd.buttons = data[3];
                ESP_LOGI(TAG_BLE, "BLE->VR2 JOY: x=%d y=%d btn=0x%02X (checksum OK)", 
                         cmd.x, cmd.y, cmd.buttons);
            } else {
                ESP_LOGW(TAG_BLE, "SET_JOY: need 5 bytes, got %d", len);
                return;
            }
            break;
            
        case BLE_CMD_START:
            /* Format: [0x02][checksum] = 2 bytes */
            cmd.cmd = VR2_CMD_START;
            ESP_LOGI(TAG_BLE, "BLE->VR2 START (checksum OK)");
            break;
            
        case BLE_CMD_SHUTDOWN:
            /* Format: [0x03][checksum] = 2 bytes */
            cmd.cmd = VR2_CMD_SHUTDOWN;
            ESP_LOGI(TAG_BLE, "BLE->VR2 SHUTDOWN (checksum OK)");
            break;
            
        case BLE_CMD_SPEED_PLUS:
            /* Format: [0x04][checksum] = 2 bytes */
            cmd.cmd = VR2_CMD_SPEED_PLUS;
            ESP_LOGI(TAG_BLE, "BLE->VR2 SPEED+ (checksum OK)");
            break;
            
        case BLE_CMD_SPEED_MINUS:
            /* Format: [0x05][checksum] = 2 bytes */
            cmd.cmd = VR2_CMD_SPEED_MINUS;
            ESP_LOGI(TAG_BLE, "BLE->VR2 SPEED- (checksum OK)");
            break;
            
        default:
            ESP_LOGW(TAG_BLE, "Unknown BLE cmd: 0x%02X", cmd_type);
            return;
    }
    
    if (cmd.cmd != VR2_CMD_NONE) {
        esp_err_t err = vr2_post_command(cmd);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_BLE, "vr2_post_command failed: %s", esp_err_to_name(err));
        }
    }
}

/**
 * Process legacy 7-byte joystick format (CHART_data_TX)
 * Legacy format does NOT include checksum for backward compatibility
 */
static void ble_process_legacy_joystick(const uint8_t *data, size_t len)
{
    if (len != sizeof(struct CHART_data_TX)) return;
    
    struct CHART_data_TX *joy = (struct CHART_data_TX *)data;
    
    vr2_command_t cmd = {0};
    cmd.cmd = VR2_CMD_SET_JOY;
    
    /* Scale from int16 to int8 range */
    int x = joy->uJoy_x;
    int y = joy->uJoy_y;
    
    /* Clamp to -127..+127 */
    if (x > 127) x = 127;
    if (x < -127) x = -127;
    if (y > 127) y = 127;
    if (y < -127) y = -127;
    
    cmd.x = (int8_t)x;
    cmd.y = (int8_t)y;
    cmd.buttons = joy->buttons;
    
    ESP_LOGI(TAG_BLE, "BLE->VR2 Legacy JOY: x=%d y=%d btn=0x%02X (no checksum)", 
             cmd.x, cmd.y, cmd.buttons);
    
    esp_err_t err = vr2_post_command(cmd);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_BLE, "vr2_post_command failed: %s", esp_err_to_name(err));
    }
}

/*============================================================================
 * TELEMETRY TASK - VR2 -> BLE
 *============================================================================*/

/**
 * Task to send ECU telemetry to BLE client
 * Now includes 5 debounced switches and checksum
 */
static void ble_telemetry_task(void *pvParameter)
{
    vr2_state_t state = {0};
    vr2_state_t last_state = {0};
    uint8_t last_switches = 0;
    uint8_t telemetry_data[5];  /* state, speed, battery, switches, checksum */
    
    ESP_LOGI(TAG_BLE, "Telemetry task started");
    
    /* Initialize the 5 telemetry switches */
    tele_sw_init();
    
    while (1) {
        /* Check if VR2 state queue exists */
        if (vr2_state_queue == NULL) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        
        /* Get latest state from VR2 */
        vr2_state_t tmp;
        while (xQueueReceive(vr2_state_queue, &tmp, 0) == pdTRUE) {
            state = tmp;
        }
        
        /* Read debounced switch states */
        uint8_t switches = tele_sw_read_debounced();
        
        /* Send if state or switches changed */
        if (state.state != last_state.state || 
            state.speed != last_state.speed ||
            state.battery != last_state.battery ||
            switches != last_switches) {
            
            last_state = state;
            last_switches = switches;
            
            /* Check if client is connected */
            EventBits_t flags = xEventGroupGetBits(BLE_event_group);
            if (flags & BLECONNECTED_BIT) {
                
                /* Pack telemetry data */
                telemetry_data[0] = (uint8_t)state.state;
                telemetry_data[1] = state.speed;
                telemetry_data[2] = state.battery;
                telemetry_data[3] = switches;
                /* Calculate checksum: (sum of bytes 0-3) XOR 0xFF */
                telemetry_data[4] = ble_calc_checksum(telemetry_data, 4);
                
                ESP_LOGI(TAG_BLE, "VR2->BLE Telemetry: state=%d speed=%d batt=%d sw=0x%02X chk=0x%02X",
                         state.state, state.speed, state.battery, switches, telemetry_data[4]);
                
                /* Send notification to client */
                esp_ble_gatts_send_indicate(
                    Joystick_profile_tab[0].gatts_if,
                    clientsConnection.joy_connid,
                    BLE_wheels_handle_table[IDX_CHAR_VAL_TELEMETRIA],
                    sizeof(telemetry_data),
                    telemetry_data,
                    false  /* false = notification, true = indication */
                );
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); /* 20Hz update rate for better debounce */
    }
}

/*============================================================================
 * HELPER FUNCTIONS
 *============================================================================*/

static char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
    char *key_str = NULL;
    switch(key_type) {
        case ESP_LE_KEY_NONE:   key_str = "ESP_LE_KEY_NONE"; break;
        case ESP_LE_KEY_PENC:   key_str = "ESP_LE_KEY_PENC"; break;
        case ESP_LE_KEY_PID:    key_str = "ESP_LE_KEY_PID"; break;
        case ESP_LE_KEY_PCSRK:  key_str = "ESP_LE_KEY_PCSRK"; break;
        case ESP_LE_KEY_PLK:    key_str = "ESP_LE_KEY_PLK"; break;
        case ESP_LE_KEY_LLK:    key_str = "ESP_LE_KEY_LLK"; break;
        case ESP_LE_KEY_LENC:   key_str = "ESP_LE_KEY_LENC"; break;
        case ESP_LE_KEY_LID:    key_str = "ESP_LE_KEY_LID"; break;
        case ESP_LE_KEY_LCSRK:  key_str = "ESP_LE_KEY_LCSRK"; break;
        default:                key_str = "INVALID BLE KEY TYPE"; break;
    }
    return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
    char *auth_str = NULL;
    switch(auth_req) {
        case ESP_LE_AUTH_NO_BOND:        auth_str = "ESP_LE_AUTH_NO_BOND"; break;
        case ESP_LE_AUTH_BOND:           auth_str = "ESP_LE_AUTH_BOND"; break;
        case ESP_LE_AUTH_REQ_MITM:       auth_str = "ESP_LE_AUTH_REQ_MITM"; break;
        case ESP_LE_AUTH_REQ_BOND_MITM:  auth_str = "ESP_LE_AUTH_REQ_BOND_MITM"; break;
        case ESP_LE_AUTH_REQ_SC_ONLY:    auth_str = "ESP_LE_AUTH_REQ_SC_ONLY"; break;
        case ESP_LE_AUTH_REQ_SC_BOND:    auth_str = "ESP_LE_AUTH_REQ_SC_BOND"; break;
        case ESP_LE_AUTH_REQ_SC_MITM:    auth_str = "ESP_LE_AUTH_REQ_SC_MITM"; break;
        case ESP_LE_AUTH_REQ_SC_MITM_BOND: auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND"; break;
        default:                         auth_str = "INVALID BLE AUTH REQ"; break;
    }
    ESP_LOGI(GATTS_TABLE_TAG, "%s", auth_str);
    return auth_str;
}

static void show_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    ESP_LOGI(GATTS_TABLE_TAG, "Bonded devices number: %d", dev_num);
    for (int i = 0; i < dev_num; i++) {
        ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, (void *)dev_list[i].bd_addr, sizeof(esp_bd_addr_t));
    }
    free(dev_list);
}

static void __attribute__((unused)) remove_all_bonded_devices(void)
{
    int dev_num = esp_ble_get_bond_device_num();
    esp_ble_bond_dev_t *dev_list = (esp_ble_bond_dev_t *)malloc(sizeof(esp_ble_bond_dev_t) * dev_num);
    esp_ble_get_bond_device_list(&dev_num, dev_list);
    for (int i = 0; i < dev_num; i++) {
        esp_ble_remove_bond_device(dev_list[i].bd_addr);
    }
    free(dev_list);
}

/*============================================================================
 * GAP EVENT HANDLER
 *============================================================================*/

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {

#ifdef EXTENDED_ADVERSING_5
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT status %d", param->ext_adv_set_params.status);
        esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(ext_adv_raw_data), &ext_adv_raw_data[0]);
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT status %d", param->ext_adv_data_set.status);
        esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT, status = %d", param->ext_adv_start.status);
        break;
    case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT, status = %d", param->ext_adv_stop.status);
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed");
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Advertising started");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
        }
        break;
#endif

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status=%d, min_int=%d, max_int=%d, conn_int=%d, latency=%d, timeout=%d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:
        break;

    case ESP_GAP_BLE_OOB_REQ_EVT:
        break;

    case ESP_GAP_BLE_LOCAL_IR_EVT:
        break;

    case ESP_GAP_BLE_LOCAL_ER_EVT:
        break;

    case ESP_GAP_BLE_NC_REQ_EVT:
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GAP_BLE_NC_REQ_EVT, passkey=%d", (unsigned int)param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "passkey=%d", (unsigned int)param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_KEY_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "key type=%s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTS_TABLE_TAG, "remote BD_ADDR: %08x%04x",
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTS_TABLE_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTS_TABLE_TAG, "pair status = %s", param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTS_TABLE_TAG, "fail reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "auth mode = %s", esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
#ifdef BLEPASSORD
            connected(Joystick_profile_tab[JOYSTICK_PROFILE_APP_IDX].gatts_if, NULL);
#endif
        }
        show_bonded_devices();
        break;
    }
    case ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT:
        ESP_LOGD(GATTS_TABLE_TAG, "ESP_GAP_BLE_REMOVE_BOND_DEV_COMPLETE_EVT status=%d", param->remove_bond_dev_cmpl.status);
        break;

    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TABLE_TAG, "config local privacy failed, error=%x", param->local_privacy_cmpl.status);
            break;
        }
#ifdef EXTENDED_ADVERSING_5
        esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params_2M);
#else
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error=%x", raw_adv_ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        raw_adv_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_adv_ret) {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp failed, error=%x", raw_adv_ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
        break;

    default:
        break;
    }
}

/*============================================================================
 * GATTS PROFILE EVENT HANDLER
 *============================================================================*/

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, 
                                        esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_REG_EVT, status=%d, app_id=%d",
                     param->reg.status, param->reg.app_id);
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_local_privacy(true);
            esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT, handle=%d", param->read.handle);
            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG_BLE, "BLE WRITE: handle=%d, len=%d", 
                     param->write.handle, param->write.len);
            ESP_LOG_BUFFER_HEX(TAG_BLE, param->write.value, param->write.len);
            
            /* Check if write is to characteristic B (command input) */
            if (BLE_wheels_handle_table[IDX_CHAR_VAL_B] == param->write.handle) {
                
                /* Check for legacy 7-byte joystick format (no checksum) */
                if (param->write.len == sizeof(struct CHART_data_TX)) {
                    ble_process_legacy_joystick(param->write.value, param->write.len);
                }
                /* New command format with checksum */
                else if (param->write.len >= 2) {
                    ble_process_command(param->write.value, param->write.len);
                }
            }
            /* Also accept writes to characteristic A for backwards compatibility */
            else if (BLE_wheels_handle_table[IDX_CHAR_VAL_A] == param->write.handle) {
                if (param->write.len >= 2) {
                    ble_process_command(param->write.value, param->write.len);
                }
            }
            break;

        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->write.value, param->write.len);
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU=%d", param->mtu.mtu);
            break;

        case ESP_GATTS_CONF_EVT:
            break;

        case ESP_GATTS_UNREG_EVT:
            break;

        case ESP_GATTS_DELETE_EVT:
            break;

        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                     param->start.status, param->start.service_handle);
            break;

        case ESP_GATTS_STOP_EVT:
            break;

        case ESP_GATTS_CONNECT_EVT:
#ifdef BLEPASSORD
            clientsConnection.login_connid = param->connect.conn_id;
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT ID %d", param->connect.conn_id);
            esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
#else
            connected(gatts_if, param);
#endif
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason 0x%x", 
                     param->disconnect.reason);
            esp_ble_gatts_close(gatts_if, clientsConnection.login_connid);
            clientsConnection.login_connid = 0xFF;
            if (param->disconnect.reason < 300) {
                disconnected(param);
            }
            break;

        case ESP_GATTS_OPEN_EVT:
            break;

        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;

        case ESP_GATTS_CLOSE_EVT:
            break;

        case ESP_GATTS_LISTEN_EVT:
            break;

        case ESP_GATTS_CONGEST_EVT:
            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "The number handle = %x", param->add_attr_tab.num_handle);
            if (param->create.status == ESP_GATT_OK) {
                if (param->add_attr_tab.num_handle == HRS_IDX_NB) {
                    memcpy(BLE_wheels_handle_table, param->add_attr_tab.handles,
                           sizeof(BLE_wheels_handle_table));
                    esp_ble_gatts_start_service(BLE_wheels_handle_table[IDX_SVC]);
                } else {
                    ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) "
                             "doesn't equal to HRS_IDX_NB(%d)",
                             param->add_attr_tab.num_handle, HRS_IDX_NB);
                }
            } else {
                ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code = %x", 
                         param->create.status);
            }
            break;
        }

        default:
            break;
    }
}

/*============================================================================
 * GATTS EVENT HANDLER
 *============================================================================*/

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            Joystick_profile_tab[JOYSTICK_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        int idx;
        for (idx = 0; idx < JOYSTICK_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE ||
                    gatts_if == Joystick_profile_tab[idx].gatts_if) {
                if (Joystick_profile_tab[idx].gatts_cb) {
                    Joystick_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

/*============================================================================
 * CONNECTION HANDLERS
 *============================================================================*/

void connected(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (clientsConnection.numero_connessioni < 1) {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT ID %d", param->connect.conn_id);
        ESP_LOGI(GATTS_TABLE_TAG, "CONNECT ADDRESS= %02x:%02x:%02x:%02x:%02x:%02x",
                 param->connect.remote_bda[0], param->connect.remote_bda[1],
                 param->connect.remote_bda[2], param->connect.remote_bda[3],
                 param->connect.remote_bda[4], param->connect.remote_bda[5]);

        clientsConnection.joy_en = 1;
        clientsConnection.joy_connid = param->connect.conn_id;

        xEventGroupSetBits(BLE_event_group, BLECONNECTED_BIT_JOYSTICK);
        xEventGroupSetBits(BLE_event_group, BLECONNECTED_BIT);

        ESP_LOGI(GATTS_TABLE_TAG, "CLIENT ******CONNECTED******");
        clientsConnection.numero_connessioni++;

#ifdef MULTICONNECTION
#ifdef EXTENDED_ADVERSING_5
        esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
#else
        esp_ble_gap_start_advertising(&adv_params);
#endif
#endif
    } else {
        esp_ble_gatts_close(gatts_if, param->connect.conn_id);
    }
}

void disconnected(esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason 0x%x", param->disconnect.reason);

    xEventGroupClearBits(BLE_event_group, BLECONNECTED_BIT_JOYSTICK);

    if (clientsConnection.numero_connessioni > 0) {
        clientsConnection.numero_connessioni--;
    }

    clientsConnection.joy_en = 0;
    clientsConnection.joy_connid = 0xFF;

    if ((clientsConnection.smartphone_en == 0) && (clientsConnection.joy_en == 0)) {
        xEventGroupClearBits(BLE_event_group, BLECONNECTED_BIT);
    }

#ifdef EXTENDED_ADVERSING_5
    esp_ble_gap_ext_adv_start(NUM_EXT_ADV_SET, &ext_adv[0]);
#else
    esp_ble_gap_start_advertising(&adv_params);
#endif

    ESP_LOGI(GATTS_TABLE_TAG, "CLIENT ******DISCONNECTED******");
}

/*============================================================================
 * SEND NOTIFICATION TASK
 *============================================================================*/

void Send_BLE_Notification_Task(void *pvParameter)
{
    uint8_t data[101];
    size_t xReceivedBytes;

    while (1) {
        xReceivedBytes = xMessageBufferReceive(BLE_SendNotification_Queue,
                                               &data[0], 100, portMAX_DELAY);

        ESP_LOGI(GATTS_TABLE_TAG, "Send notification: %d bytes", xReceivedBytes);

        if (xReceivedBytes > 1) {
            EventBits_t flags = xEventGroupGetBits(BLE_event_group);

            if (flags & BLECONNECTED_BIT) {
                ESP_LOGI(GATTS_TABLE_TAG, "Sending %d bytes to BLE", xReceivedBytes);

                esp_ble_gatts_send_indicate(
                    Joystick_profile_tab[0].gatts_if,
                    clientsConnection.joy_connid,
                    BLE_wheels_handle_table[IDX_CHAR_VAL_TELEMETRIA],
                    xReceivedBytes,
                    &data[0],
                    false
                );
            }
        }
    }
}

/*============================================================================
 * BLE SERVICE START
 *============================================================================*/

void BLE_Service_Start(void)
{
    esp_err_t ret;

    clientsConnection.smartphone_en = 0;
    clientsConnection.joy_en = 0;
    clientsConnection.smartphone_connid = 0xFF;
    clientsConnection.joy_connid = 0xFF;
    clientsConnection.numero_connessioni = 0;

    /* Initialize NVS */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_JOYSTICK_APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

#ifdef BLEPASSORD
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_ONLY;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = 123456;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
#endif

    /* Create event group and message buffer */
    BLE_event_group = xEventGroupCreate();
    BLE_SendNotification_Queue = xMessageBufferCreate(100);

    /* Create notification task */
    xTaskCreate(&Send_BLE_Notification_Task, "BLE_Notify", 4096, NULL, tskIDLE_PRIORITY, NULL);
    
    /* Create telemetry task for VR2 -> BLE */
    xTaskCreate(&ble_telemetry_task, "BLE_Telemetry", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);

    ESP_LOGI(TAG_BLE, "BLE Service started - VR2 Bridge ready");
    ESP_LOGI(TAG_BLE, "Service UUID: da34-3711-d712-3230-8842-f9de-a033-233d");
    ESP_LOGI(TAG_BLE, "Telemetry format: [state][speed][battery][switches][checksum]");
    ESP_LOGI(TAG_BLE, "Command format: [cmd][data...][checksum]");
}

/*============================================================================
 * BLE SERVICE STOP
 *============================================================================*/

void BLE_Service_Stop(void)
{
    ESP_LOGI(TAG_BLE, "Stopping BLE Service...");
    
    /* Stop advertising */
#ifdef EXTENDED_ADVERSING_5
    uint8_t ext_adv_inst[1] = {EXT_ADV_HANDLE};
    esp_ble_gap_ext_adv_stop(NUM_EXT_ADV_SET, ext_adv_inst);
#else
    esp_ble_gap_stop_advertising();
#endif
    
    /* Disconnect any connected clients */
    if (clientsConnection.joy_en) {
        esp_ble_gatts_close(Joystick_profile_tab[0].gatts_if, clientsConnection.joy_connid);
    }
    
    /* Clear event group bits */
    if (BLE_event_group) {
        xEventGroupClearBits(BLE_event_group, BLECONNECTED_BIT | BLECONNECTED_BIT_JOYSTICK);
    }
    
    /* Small delay to allow disconnect to complete */
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /* Unregister GATT application */
    esp_ble_gatts_app_unregister(Joystick_profile_tab[0].gatts_if);
    
    /* Disable and deinit Bluedroid */
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    
    /* Disable and deinit BT controller */
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    
    /* Reset connection state */
    clientsConnection.smartphone_en = 0;
    clientsConnection.joy_en = 0;
    clientsConnection.smartphone_connid = 0xFF;
    clientsConnection.joy_connid = 0xFF;
    clientsConnection.numero_connessioni = 0;
    
    ESP_LOGI(TAG_BLE, "BLE Service stopped");
}

/*============================================================================
 * BLE CONNECTION CHECK
 *============================================================================*/

bool BLE_Is_Connected(void)
{
    if (BLE_event_group == NULL) {
        return false;
    }
    EventBits_t flags = xEventGroupGetBits(BLE_event_group);
    return (flags & BLECONNECTED_BIT) != 0;
}