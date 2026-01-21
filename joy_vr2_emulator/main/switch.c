/**
 * @file switch.c
 * @brief GPIO Switch/Button Handler Implementation for ESP32-S3
 */

#include "switch.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SWITCH";

/*============================================================================
 * PRIVATE DATA
 *============================================================================*/

/* GPIO pin array for easy iteration */
static const gpio_num_t sw_pins[SW_NUM_BUTTONS] = {
    SW_SW2DW_PIN,   /* GPIO37 - sw2dw */
    SW_SW2UP_PIN,   /* GPIO36 - sw2up */
    SW_SW2R_PIN,    /* GPIO35 - sw2r */
    SW_SW2C_PIN,    /* GPIO0  - sw2c (STRAPPING!) */
    SW_SWL2_PIN,    /* GPIO45 - swl2 (STRAPPING!) */
    SW_SWR_PIN,     /* GPIO48 - swr */
    SW_SEC_PIN,     /* GPIO47 - sec */
    SW_SWL_PIN,     /* GPIO21 - swl */
};

/* Button names for debugging */
static const char *sw_names[SW_NUM_BUTTONS] = {
    "sw2dw",
    "sw2up",
    "sw2r",
    "sw2c",
    "swl2",
    "swr",
    "sec",
    "swl",
};

/*============================================================================
 * INITIALIZATION
 *============================================================================*/

esp_err_t sw_init(void)
{
    esp_err_t ret = ESP_OK;
    
    ESP_LOGI(TAG, "Initializing %d switch GPIOs with pull-up...", SW_NUM_BUTTONS);
    
    /* Warning about strapping pins */
    ESP_LOGW(TAG, "WARNING: GPIO0 (sw2c) and GPIO45 (swl2) are strapping pins!");
    ESP_LOGW(TAG, "Do not press these buttons during boot/reset!");

    gpio_config_t io_conf = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    /* Build pin bitmask */
    for (int i = 0; i < SW_NUM_BUTTONS; i++) {
        io_conf.pin_bit_mask |= (1ULL << sw_pins[i]);
    }

    /* Configure all pins at once */
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Log configured pins */
    for (int i = 0; i < SW_NUM_BUTTONS; i++) {
        ESP_LOGI(TAG, "  %s -> GPIO%d", sw_names[i], sw_pins[i]);
    }

    ESP_LOGI(TAG, "Switch initialization complete");
    return ESP_OK;
}

/*============================================================================
 * BUTTON READING FUNCTIONS
 *============================================================================*/

bool sw_read_button(sw_button_idx_t idx)
{
    if (idx >= SW_NUM_BUTTONS) {
        ESP_LOGW(TAG, "Invalid button index: %d", idx);
        return false;
    }
    
    /* Active LOW: gpio_get_level returns 0 when pressed */
    return (gpio_get_level(sw_pins[idx]) == 0);
}

uint8_t sw_read_all_raw(void)
{
    uint8_t state = 0;
    
    for (int i = 0; i < SW_NUM_BUTTONS; i++) {
        /* Active LOW: invert the logic */
        if (gpio_get_level(sw_pins[i]) == 0) {
            state |= (1 << i);
        }
    }
    
    return state;
}

void sw_read_all(sw_state_t *state)
{
    if (state == NULL) {
        return;
    }
    
    state->raw = sw_read_all_raw();
    
    /* Unpack to individual booleans */
    state->sw2dw = (state->raw & SW_BTN_SW2DW) != 0;
    state->sw2up = (state->raw & SW_BTN_SW2UP) != 0;
    state->sw2r  = (state->raw & SW_BTN_SW2R) != 0;
    state->sw2c  = (state->raw & SW_BTN_SW2C) != 0;
    state->swl2  = (state->raw & SW_BTN_SWL2) != 0;
    state->swr   = (state->raw & SW_BTN_SWR) != 0;
    state->sec   = (state->raw & SW_BTN_SEC) != 0;
    state->swl   = (state->raw & SW_BTN_SWL) != 0;
}

void sw_read_all_debounced(sw_state_t *state, uint32_t debounce_ms)
{
    if (state == NULL) {
        return;
    }
    
    /* First reading */
    uint8_t first = sw_read_all_raw();
    
    /* Wait for debounce */
    vTaskDelay(pdMS_TO_TICKS(debounce_ms));
    
    /* Second reading - only accept buttons that are still pressed */
    uint8_t second = sw_read_all_raw();
    
    /* AND the two readings for stable state */
    state->raw = first & second;
    
    /* Unpack to individual booleans */
    state->sw2dw = (state->raw & SW_BTN_SW2DW) != 0;
    state->sw2up = (state->raw & SW_BTN_SW2UP) != 0;
    state->sw2r  = (state->raw & SW_BTN_SW2R) != 0;
    state->sw2c  = (state->raw & SW_BTN_SW2C) != 0;
    state->swl2  = (state->raw & SW_BTN_SWL2) != 0;
    state->swr   = (state->raw & SW_BTN_SWR) != 0;
    state->sec   = (state->raw & SW_BTN_SEC) != 0;
    state->swl   = (state->raw & SW_BTN_SWL) != 0;
}

const char* sw_get_button_name(sw_button_idx_t idx)
{
    if (idx >= SW_NUM_BUTTONS) {
        return "UNKNOWN";
    }
    return sw_names[idx];
}
