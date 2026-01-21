/**
 * @file switch.h
 * @brief GPIO Switch/Button Handler for ESP32-S3
 * 
 * 8 buttons configured with internal pull-up resistors.
 * Active LOW logic (pressed = 0, released = 1)
 * 
 * WARNING: GPIO0 and GPIO45 are strapping pins!
 * - GPIO0: If LOW during boot -> download mode
 * - GPIO45: VDD_SPI voltage selection during boot
 * Ensure buttons are not pressed during power-on/reset.
 */

#ifndef SWITCH_H
#define SWITCH_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

/*============================================================================
 * GPIO PIN DEFINITIONS
 *============================================================================*/

#define SW_SW2DW_PIN    GPIO_NUM_37   /* sw2dw - Down */
#define SW_SW2UP_PIN    GPIO_NUM_36   /* sw2up - Up */
#define SW_SW2R_PIN     GPIO_NUM_35   /* sw2r  - Right */
#define SW_SW2C_PIN     GPIO_NUM_0    /* sw2c  - Center (STRAPPING PIN!) */
#define SW_SWL2_PIN     GPIO_NUM_45   /* swl2  - Left 2 (STRAPPING PIN!) */
#define SW_SWR_PIN      GPIO_NUM_48   /* swr   - Right */
#define SW_SEC_PIN      GPIO_NUM_47   /* sec   - Secondary */
#define SW_SWL_PIN      GPIO_NUM_21   /* swl   - Left */

#define SW_NUM_BUTTONS  8

/*============================================================================
 * BUTTON INDEX ENUMERATION
 *============================================================================*/

typedef enum {
    SW_IDX_SW2DW = 0,   /* GPIO37 */
    SW_IDX_SW2UP,       /* GPIO36 */
    SW_IDX_SW2R,        /* GPIO35 */
    SW_IDX_SW2C,        /* GPIO0  */
    SW_IDX_SWL2,        /* GPIO45 */
    SW_IDX_SWR,         /* GPIO48 */
    SW_IDX_SEC,         /* GPIO47 */
    SW_IDX_SWL,         /* GPIO21 */
} sw_button_idx_t;

/*============================================================================
 * BUTTON STATE FLAGS (bitmask)
 *============================================================================*/

#define SW_BTN_SW2DW    (1 << SW_IDX_SW2DW)   /* 0x01 */
#define SW_BTN_SW2UP    (1 << SW_IDX_SW2UP)   /* 0x02 */
#define SW_BTN_SW2R     (1 << SW_IDX_SW2R)    /* 0x04 */
#define SW_BTN_SW2C     (1 << SW_IDX_SW2C)    /* 0x08 */
#define SW_BTN_SWL2     (1 << SW_IDX_SWL2)    /* 0x10 */
#define SW_BTN_SWR      (1 << SW_IDX_SWR)     /* 0x20 */
#define SW_BTN_SEC      (1 << SW_IDX_SEC)     /* 0x40 */
#define SW_BTN_SWL      (1 << SW_IDX_SWL)     /* 0x80 */

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

/**
 * @brief Complete switch state structure
 */
typedef struct {
    uint8_t raw;            /* Raw bitmask of all buttons (1=pressed) */
    bool sw2dw;             /* Down button */
    bool sw2up;             /* Up button */
    bool sw2r;              /* Right button */
    bool sw2c;              /* Center button */
    bool swl2;              /* Left 2 button */
    bool swr;               /* Right button */
    bool sec;               /* Secondary button */
    bool swl;               /* Left button */
} sw_state_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize all switch GPIOs with internal pull-up
 * @return ESP_OK on success
 */
esp_err_t sw_init(void);

/**
 * @brief Read single button state
 * @param idx Button index (sw_button_idx_t)
 * @return true if pressed (active LOW), false if released
 */
bool sw_read_button(sw_button_idx_t idx);

/**
 * @brief Read all buttons as bitmask
 * @return Bitmask where bit=1 means button pressed
 */
uint8_t sw_read_all_raw(void);

/**
 * @brief Read all buttons into state structure
 * @param state Pointer to state structure to fill
 */
void sw_read_all(sw_state_t *state);

/**
 * @brief Read all buttons with software debounce
 * @param state Pointer to state structure to fill
 * @param debounce_ms Debounce time in milliseconds (typical: 10-50ms)
 */
void sw_read_all_debounced(sw_state_t *state, uint32_t debounce_ms);

/**
 * @brief Get button name string
 * @param idx Button index
 * @return Button name string
 */
const char* sw_get_button_name(sw_button_idx_t idx);

#endif /* SWITCH_H */
