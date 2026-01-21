/**
 * @file led.c
 * @brief RGB LED Control Implementation for ESP32-S3
 * 
 * Active LOW logic: GPIO HIGH = LED OFF, GPIO LOW = LED ON
 */

#include "led.h"
#include "esp_log.h"

static const char *TAG = "LED";

/*============================================================================
 * INITIALIZATION
 *============================================================================*/

esp_err_t led_init(void)
{
    ESP_LOGI(TAG, "Initializing RGB LED GPIOs...");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_RED_PIN) | 
                        (1ULL << LED_GREEN_PIN) | 
                        (1ULL << LED_BLUE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure LED GPIOs: %s", esp_err_to_name(ret));
        return ret;
    }
    
    /* Start with all LEDs off (HIGH = OFF for active low) */
    gpio_set_level(LED_RED_PIN, 1);
    gpio_set_level(LED_GREEN_PIN, 1);
    gpio_set_level(LED_BLUE_PIN, 1);
    
    ESP_LOGI(TAG, "LED init complete: R=GPIO%d, G=GPIO%d, B=GPIO%d",
             LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN);
    
    return ESP_OK;
}

/*============================================================================
 * LED CONTROL FUNCTIONS
 *============================================================================*/

void led_set(bool red, bool green, bool blue)
{
    /* Active LOW: true -> GPIO LOW (on), false -> GPIO HIGH (off) */
    gpio_set_level(LED_RED_PIN, red ? 0 : 1);
    gpio_set_level(LED_GREEN_PIN, green ? 0 : 1);
    gpio_set_level(LED_BLUE_PIN, blue ? 0 : 1);
}

void led_set_color(led_color_t color)
{
    switch (color) {
        case LED_COLOR_OFF:
            led_set(false, false, false);
            break;
        case LED_COLOR_RED:
            led_set(true, false, false);
            break;
        case LED_COLOR_GREEN:
            led_set(false, true, false);
            break;
        case LED_COLOR_BLUE:
            led_set(false, false, true);
            break;
        case LED_COLOR_YELLOW:
            led_set(true, true, false);
            break;
        case LED_COLOR_CYAN:
            led_set(false, true, true);
            break;
        case LED_COLOR_MAGENTA:
            led_set(true, false, true);
            break;
        case LED_COLOR_WHITE:
            led_set(true, true, true);
            break;
        default:
            led_set(false, false, false);
            break;
    }
}

void led_off(void)
{
    led_set(false, false, false);
}

void led_red(void)
{
    led_set(true, false, false);
}

void led_green(void)
{
    led_set(false, true, false);
}

void led_blue(void)
{
    led_set(false, false, true);
}
