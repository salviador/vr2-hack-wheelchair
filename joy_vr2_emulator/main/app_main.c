/*
 * VR2 PG Drives Wheelchair Controller - BLE Bridge
 * 
 * Main application with power control:
 * - Waits for SEC button to start system (if ENABLE_SWITCH_START defined)
 * - Or starts immediately on power-on (if ENABLE_SWITCH_START not defined)
 * - Controls relay, LEDs, VR2, and BLE based on state
 * - SEC button toggles system on/off
 * - SWL/SWR control speed when VR2 is running
 * - Blue LED indicates BLE connection
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "vr2_pgdrives.h"
#include "bleService.h"
#include "switch.h"
#include "led.h"

/*============================================================================
 * CONFIGURATION
 *============================================================================*/

/**
 * ENABLE_SWITCH_START - Controls startup behavior
 * 
 * If DEFINED:   System waits for SEC button press to start VR2 and BLE
 *               SEC button can toggle system ON/OFF
 * 
 * If COMMENTED: System starts VR2 and BLE immediately on power-on
 *               SEC button has no effect (always running)
 */
//#define ENABLE_SWITCH_START

/*============================================================================*/

#define BUF_SIZE 64

static const char *TAGC = "MAIN";

/*============================================================================
 * RELAY CONTROL
 *============================================================================*/

#define RELAY_PIN   GPIO_NUM_14

static void relay_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(RELAY_PIN, 0);  /* Start OFF */
}

static void relay_on(void)
{
    gpio_set_level(RELAY_PIN, 1);
    ESP_LOGI(TAGC, "Relay ON");
}

static void relay_off(void)
{
    gpio_set_level(RELAY_PIN, 0);
    ESP_LOGI(TAGC, "Relay OFF");
}

/*============================================================================
 * SYSTEM STATE MACHINE
 *============================================================================*/

typedef enum {
    SYS_STATE_OFF = 0,      /* System off, waiting for SEC button */
    SYS_STATE_STARTING,     /* Starting up */
    SYS_STATE_RUNNING,      /* VR2 and BLE active */
    SYS_STATE_STOPPING,     /* Shutting down */
} sys_state_t;

static volatile sys_state_t sys_state = SYS_STATE_OFF;

/* ECU state names for display */
static const char *vr2_ecu_state_str[] = {
    "IDLE",
    "INIT",
    "RUN",
    "ERROR",
    "SHUTDOWN"
};

/*============================================================================
 * CONSOLE UART0 INITIALIZATION
 *============================================================================*/

static void console_uart0_init(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));

    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &cfg));
}

/*============================================================================
 * HELPER FUNCTIONS
 *============================================================================*/

static int8_t clamp100(int v)
{
    if (v > 100) return 100;
    if (v < -100) return -100;
    return (int8_t)v;
}

static vr2_ecu_state_t get_current_vr2_state(void)
{
    vr2_state_t state = {0};
    vr2_state_t tmp;

    if (!vr2_state_queue) {
        return VR2_ECU_IDLE;
    }

    /* Peek at queue without removing (actually we need to receive and put back) */
    while (xQueueReceive(vr2_state_queue, &tmp, 0) == pdTRUE) {
        state = tmp;
    }
    /* Put last state back */
    if (vr2_state_queue) {
        xQueueOverwrite(vr2_state_queue, &state);
    }
    
    return state.state;
}

static inline void post_simple_cmd(uint8_t id)
{
    vr2_command_t cmd = {0};
    cmd.cmd = id;
    esp_err_t e = vr2_post_command(cmd);
    ESP_LOGI(TAGC, "CMD %u -> %s", (unsigned)id, esp_err_to_name(e));
}

static inline void post_joy(int8_t x, int8_t y)
{
    vr2_command_t cmd = {0};
    cmd.cmd = VR2_CMD_SET_JOY;
    cmd.x = x;
    cmd.y = y;
    esp_err_t e = vr2_post_command(cmd);
    ESP_LOGI(TAGC, "JOY x=%d y=%d -> %s", (int)x, (int)y, esp_err_to_name(e));
}

/*============================================================================
 * BUTTON DEBOUNCE HELPERS
 *============================================================================*/

#ifdef ENABLE_SWITCH_START
static bool wait_button_release(sw_button_idx_t btn, uint32_t timeout_ms)
{
    uint32_t start = xTaskGetTickCount();
    while (sw_read_button(btn)) {
        vTaskDelay(pdMS_TO_TICKS(20));
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms)) {
            return false;  /* Timeout */
        }
    }
    return true;
}
#endif

/*============================================================================
 * SYSTEM STARTUP SEQUENCE
 *============================================================================*/

static void system_startup(void)
{
    ESP_LOGI(TAGC, "=== SYSTEM STARTUP ===");
    sys_state = SYS_STATE_STARTING;
    
    /* 1. Activate relay */
    relay_on();
    
    /* 2. Wait 1 second */
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    /* 3. Turn on green LED */
    led_green();
    ESP_LOGI(TAGC, "LED: GREEN (starting)");
    
    /* 4. Start VR2 emulator */
    ESP_LOGI(TAGC, "Starting VR2 emulator...");
    vr2_emulator_start();
    
    /* Small delay to let VR2 initialize */
    vTaskDelay(pdMS_TO_TICKS(500));
    
    /* 5. Start BLE service */
    ESP_LOGI(TAGC, "Starting BLE service...");
    BLE_Service_Start();
    
#ifdef ENABLE_SWITCH_START
    /* 6. Wait for SEC button release */
    ESP_LOGI(TAGC, "Waiting for SEC button release...");
    wait_button_release(SW_IDX_SEC, 5000);
#endif
    
    sys_state = SYS_STATE_RUNNING;
    ESP_LOGI(TAGC, "=== SYSTEM RUNNING ===");
}

/*============================================================================
 * SYSTEM SHUTDOWN SEQUENCE
 *============================================================================*/

#ifdef ENABLE_SWITCH_START
static void system_shutdown(void)
{
    ESP_LOGI(TAGC, "=== SYSTEM SHUTDOWN ===");
    sys_state = SYS_STATE_STOPPING;
    
    /* 1. Check VR2 state - if RUN, send shutdown command */
    vr2_ecu_state_t ecu_state = get_current_vr2_state();
    ESP_LOGI(TAGC, "Current VR2 state: %s", 
             ecu_state < 5 ? vr2_ecu_state_str[ecu_state] : "UNKNOWN");
    
    if (ecu_state == VR2_ECU_RUN || ecu_state == VR2_ECU_INIT) {
        ESP_LOGI(TAGC, "VR2 is active, sending SHUTDOWN command...");
        post_simple_cmd(VR2_CMD_SHUTDOWN);
        
        /* Wait for shutdown to complete */
        for (int i = 0; i < 50; i++) {  /* Max 5 seconds */
            vTaskDelay(pdMS_TO_TICKS(100));
            ecu_state = get_current_vr2_state();
            if (ecu_state == VR2_ECU_IDLE || ecu_state == VR2_ECU_SHUTDOWN) {
                ESP_LOGI(TAGC, "VR2 shutdown complete");
                break;
            }
        }
    }
    
    /* 2. Stop BLE service */
    ESP_LOGI(TAGC, "Stopping BLE service...");
    BLE_Service_Stop();
    
    /* 3. Turn off relay */
    relay_off();
    
    /* 4. Turn off all LEDs */
    led_off();
    ESP_LOGI(TAGC, "LED: OFF");
    
    /* 5. Wait for SEC button release */
    ESP_LOGI(TAGC, "Waiting for SEC button release...");
    wait_button_release(SW_IDX_SEC, 5000);
    
    sys_state = SYS_STATE_OFF;
    ESP_LOGI(TAGC, "=== SYSTEM OFF ===");
}
#endif

/*============================================================================
 * LED UPDATE BASED ON STATE
 *============================================================================*/

static void update_led_state(void)
{
    static bool last_ble_connected = false;
    
    if (sys_state != SYS_STATE_RUNNING) {
        return;  /* LED handled by startup/shutdown */
    }
    
    bool ble_connected = BLE_Is_Connected();
    
    if (ble_connected != last_ble_connected) {
        last_ble_connected = ble_connected;
        
        if (ble_connected) {
            led_blue();
            ESP_LOGI(TAGC, "LED: BLUE (BLE connected)");
        } else {
            led_green();
            ESP_LOGI(TAGC, "LED: GREEN (BLE disconnected)");
        }
    }
}

/*============================================================================
 * MAIN CONTROL TASK
 *============================================================================*/

static void control_task(void *arg)
{
    (void)arg;
    
    /* Track button states for one-shot detection */
    bool swl_was_pressed = false;
    bool swr_was_pressed = false;
#ifdef ENABLE_SWITCH_START
    bool sec_was_pressed = false;
    
    ESP_LOGI(TAGC, "Control task started");
    ESP_LOGI(TAGC, "Press SEC button to start system");
#else
    ESP_LOGI(TAGC, "Control task started - Auto-start mode");
    
    /* Auto-start immediately */
    system_startup();
#endif
    
    while (1) {
        /* Read current button states */
        bool swl_pressed = sw_read_button(SW_IDX_SWL);
        bool swr_pressed = sw_read_button(SW_IDX_SWR);
#ifdef ENABLE_SWITCH_START
        bool sec_pressed = sw_read_button(SW_IDX_SEC);
#endif
        
        switch (sys_state) {
#ifdef ENABLE_SWITCH_START
            case SYS_STATE_OFF:
                /* Wait for SEC button press to start */
                if (sec_pressed && !sec_was_pressed) {
                    ESP_LOGI(TAGC, "SEC pressed - starting system");
                    system_startup();
                }
                break;
#else
            case SYS_STATE_OFF:
                /* Should not happen in auto-start mode */
                break;
#endif
                
            case SYS_STATE_RUNNING:
#ifdef ENABLE_SWITCH_START
                /* Check for SEC button to shutdown */
                if (sec_pressed && !sec_was_pressed) {
                    ESP_LOGI(TAGC, "SEC pressed - shutting down");
                    system_shutdown();
                    break;  /* Exit switch to avoid processing other buttons */
                }
#endif
                
                /* Update LED based on BLE connection */
                update_led_state();
                
                /* Handle speed buttons (one-shot) - only when VR2 is in RUN state */
                vr2_ecu_state_t ecu_state = get_current_vr2_state();
                if (ecu_state == VR2_ECU_RUN) {
                    /* SWR = Speed Plus */
                    if (swr_pressed && !swr_was_pressed) {
                        ESP_LOGI(TAGC, "SWR pressed - SPEED+");
                        post_simple_cmd(VR2_CMD_SPEED_PLUS);
                    }
                    
                    /* SWL = Speed Minus */
                    if (swl_pressed && !swl_was_pressed) {
                        ESP_LOGI(TAGC, "SWL pressed - SPEED-");
                        post_simple_cmd(VR2_CMD_SPEED_MINUS);
                    }
                }
                break;
                
            case SYS_STATE_STARTING:
            case SYS_STATE_STOPPING:
                /* Do nothing during transitions */
                break;
        }
        
        /* Update last button states */
#ifdef ENABLE_SWITCH_START
        sec_was_pressed = sec_pressed;
#endif
        swl_was_pressed = swl_pressed;
        swr_was_pressed = swr_pressed;
        
        vTaskDelay(pdMS_TO_TICKS(50));  /* 20Hz polling */
    }
}

/*============================================================================
 * CONSOLE COMMAND PROCESSOR (for debugging)
 *============================================================================*/

static int8_t forward_debug = 0;

static void Print_Status_IDF(void)
{
    vr2_state_t state = {0};
    vr2_state_t tmp;

    ESP_LOGI(TAGC, "System state: %s", 
             sys_state == SYS_STATE_OFF ? "OFF" :
             sys_state == SYS_STATE_STARTING ? "STARTING" :
             sys_state == SYS_STATE_RUNNING ? "RUNNING" :
             sys_state == SYS_STATE_STOPPING ? "STOPPING" : "UNKNOWN");
    
    ESP_LOGI(TAGC, "BLE connected: %s", BLE_Is_Connected() ? "YES" : "NO");

    if (!vr2_state_queue) {
        ESP_LOGE(TAGC, "state_queue is NULL");
        return;
    }

    /* Get latest state from queue */
    while (xQueueReceive(vr2_state_queue, &tmp, 0) == pdTRUE) {
        state = tmp;
    }
    xQueueOverwrite(vr2_state_queue, &state);

    const char *state_name = "UNKNOWN";
    if (state.state < (sizeof(vr2_ecu_state_str) / sizeof(vr2_ecu_state_str[0]))) {
        state_name = vr2_ecu_state_str[state.state];
    }

    ESP_LOGI(TAGC,
        "\n=== VR2 ECU STATE ===\n"
        "STATE   = %s (%d)\n"
        "SPEED   = %d\n"
        "BATTERY = 0x%02X\n"
        "====================",
        state_name,
        state.state,
        state.speed,
        state.battery
    );
}

static void Process_UART0_Command(uint8_t ch)
{
    /* Only allow commands when system is running */
    if (sys_state != SYS_STATE_RUNNING) {
        if (ch != '?' && ch != 'h' && ch != 'H' && ch != 's' && ch != 'S') {
#ifdef ENABLE_SWITCH_START
            ESP_LOGW(TAGC, "System not running. Press SEC button to start.");
#else
            ESP_LOGW(TAGC, "System not running yet. Please wait...");
#endif
            return;
        }
    }
    
    switch (ch) {
        case 'E':
        case 'e':
            post_simple_cmd(VR2_CMD_START);
            ESP_LOGI(TAGC, ">>> START");
            break;

        case 'X':
        case 'x':
            post_simple_cmd(VR2_CMD_SHUTDOWN);
            ESP_LOGI(TAGC, ">>> SHUTDOWN");
            break;

        case '+':
            post_simple_cmd(VR2_CMD_SPEED_PLUS);
            ESP_LOGI(TAGC, ">>> SPEED UP");
            break;

        case '-':
            post_simple_cmd(VR2_CMD_SPEED_MINUS);
            ESP_LOGI(TAGC, ">>> SPEED DOWN");
            break;

        case '.':
            forward_debug = clamp100(forward_debug + 5);
            post_joy(0, forward_debug);
            ESP_LOGI(TAGC, ">>> FWD debug=%d", (int)forward_debug);
            break;

        case ',':
            forward_debug = clamp100(forward_debug - 5);
            post_joy(0, forward_debug);
            ESP_LOGI(TAGC, ">>> FWD debug=%d", (int)forward_debug);
            break;

        case 'F':
        case 'f':
            post_joy(0, +80);
            ESP_LOGI(TAGC, ">>> FORWARD");
            break;

        case 'B':
        case 'b':
            post_joy(0, -80);
            ESP_LOGI(TAGC, ">>> BACK");
            break;

        case 'L':
        case 'l':
            post_joy(-80, 0);
            ESP_LOGI(TAGC, ">>> LEFT");
            break;

        case 'R':
        case 'r':
            post_joy(+80, 0);
            ESP_LOGI(TAGC, ">>> RIGHT");
            break;

        case 'C':
        case 'c':
            forward_debug = 0;
            post_joy(0, 0);
            ESP_LOGI(TAGC, ">>> CENTER/STOP");
            break;

        case 'S':
        case 's':
            Print_Status_IDF();
            break;

        case '?':
        case 'H':
        case 'h':
            ESP_LOGI(TAGC,
                "\n=== VR2 CONSOLE COMMANDS ===\n"
                "E = Start/restart ECU\n"
                "X = Shutdown ECU\n"
                "+ = Speed up\n"
                "- = Speed down\n"
                "F/B/L/R = Move Forward/Back/Left/Right\n"
                "C = Center/stop\n"
                "S = Show status\n"
                ". , = Forward debug +5/-5\n"
                "? = This help\n"
                "============================\n"
                "\n=== BUTTONS ===\n"
#ifdef ENABLE_SWITCH_START
                "SEC = Toggle system ON/OFF\n"
#endif
                "SWR = Speed+ (when VR2 running)\n"
                "SWL = Speed- (when VR2 running)\n"
                "====================");
            break;

        case '\r':
        case '\n':
            break;

        default:
            ESP_LOGW(TAGC, "Unknown command '%c' (0x%02X). Press '?' for help.", ch, ch);
            break;
    }
}

/*============================================================================
 * CONSOLE TASK
 *============================================================================*/

static void console_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAGC, "Console ready on UART0. Press '?' for help.");

    uint8_t ch;
    while (1) {
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(50));
        if (n == 1) {
            Process_UART0_Command(ch);
        }
    }
}

/*============================================================================
 * MAIN APPLICATION
 *============================================================================*/

void app_main(void)
{
    ESP_LOGI(TAGC, "");
    ESP_LOGI(TAGC, "========================================");
    ESP_LOGI(TAGC, "  VR2 PG Drives BLE Bridge v2.0");
    ESP_LOGI(TAGC, "========================================");
#ifdef ENABLE_SWITCH_START
    ESP_LOGI(TAGC, "  Mode: Button Start (SEC to start)");
#else
    ESP_LOGI(TAGC, "  Mode: Auto Start");
#endif
    ESP_LOGI(TAGC, "");

    /* Print chip info */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAGC, "Chip: %s, %d cores, WiFi%s%s%s",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
             (chip_info.features & CHIP_FEATURE_IEEE802154) ? "/802.15.4" : "");

    uint32_t flash_size;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAGC, "Flash: %lu MB %s", flash_size / (1024 * 1024),
                (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "(embedded)" : "(external)");
    }

    /* Initialize peripherals */
    //console_uart0_init();
    relay_init();
    led_init();
    sw_init();
    
    /* Start with system OFF */
    sys_state = SYS_STATE_OFF;
    led_off();
    relay_off();
    
    /* Create control task on core 0 */
    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 10, NULL, 0);
    
    /* Create console task on core 0 */
    //xTaskCreatePinnedToCore(console_task, "console_task", 4096, NULL, 5, NULL, 0);

    ESP_LOGI(TAGC, "");
    ESP_LOGI(TAGC, "System initialized!");
#ifdef ENABLE_SWITCH_START
    ESP_LOGI(TAGC, "- Press SEC button to start");
#else
    ESP_LOGI(TAGC, "- Auto-starting VR2 and BLE...");
#endif
    ESP_LOGI(TAGC, "- Console ready (press '?' for help)");
    ESP_LOGI(TAGC, "");

    /* Main loop - just heartbeat */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        ESP_LOGI(TAGC, "Heartbeat - sys_state=%d", sys_state);
    }
}
