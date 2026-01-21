/**
 * @file led.h
 * @brief RGB LED Control for ESP32-S3
 * 
 * LED connections (Active LOW - HIGH=OFF, LOW=ON):
 * - Red:   GPIO39
 * - Green: GPIO40
 * - Blue:  GPIO41
 */

#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

/*============================================================================
 * GPIO PIN DEFINITIONS
 *============================================================================*/

#define LED_RED_PIN     GPIO_NUM_39
#define LED_GREEN_PIN   GPIO_NUM_40
#define LED_BLUE_PIN    GPIO_NUM_41

/*============================================================================
 * LED COLOR DEFINITIONS
 *============================================================================*/

typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_RED,
    LED_COLOR_GREEN,
    LED_COLOR_BLUE,
    LED_COLOR_YELLOW,   /* Red + Green */
    LED_COLOR_CYAN,     /* Green + Blue */
    LED_COLOR_MAGENTA,  /* Red + Blue */
    LED_COLOR_WHITE,    /* All on */
} led_color_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize LED GPIOs
 * @return ESP_OK on success
 */
esp_err_t led_init(void);

/**
 * @brief Set individual LED state
 * @param red   true = ON, false = OFF
 * @param green true = ON, false = OFF
 * @param blue  true = ON, false = OFF
 */
void led_set(bool red, bool green, bool blue);

/**
 * @brief Set LED color using predefined colors
 * @param color LED color enum
 */
void led_set_color(led_color_t color);

/**
 * @brief Turn all LEDs off
 */
void led_off(void);

/**
 * @brief Set only red LED on
 */
void led_red(void);

/**
 * @brief Set only green LED on
 */
void led_green(void);

/**
 * @brief Set only blue LED on
 */
void led_blue(void);

#endif /* LED_H */
