#ifndef VR2_PGDRIVES_H_
#define VR2_PGDRIVES_H_

#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
uint32_t vr2_millis(void);
#define VR2_UART_NUM      UART_NUM_1
#define VR2_UART_TX_PIN   GPIO_NUM_17
#define VR2_UART_RX_PIN   GPIO_NUM_18

#define VR2_UART_BAUDRATE 38400
#define VR2_RX_BUF_SIZE   1024
#define VR2_TX_BUF_SIZE   1024

esp_err_t vr2_uart_init(void);
int vr2_uart_write(const uint8_t *data, size_t len);
int vr2_uart_read(uint8_t *data, size_t maxlen, uint32_t timeout_ms);

#define VR2_UART_EN_TX    GPIO_NUM_11
#define VR2_PUL_UP_EN     GPIO_NUM_12
#define VR2_PUL_DW_EN     GPIO_NUM_13
#define VR2_RELE_EN       GPIO_NUM_14

void vr2_init_gpio_control(void);
void vre_joy_dis_tx(void);
void vre_joy_en_tx(void);
void vre_joy_pull_no(void);
void vre_joy_pull_down(void);
void vre_joy_pull_up(void);
void vr2_set_rx_uart(void);
uint8_t vr2_get_rx_gpio(void);
void vr2_set_rx_gpio_input(void);




/* Joystick data structure */
typedef struct {
    int16_t x;              /* -127 to +127, 0 = center */
    int16_t y;              /* -127 to +127, 0 = center */
    uint8_t btn_speed_plus;
    uint8_t btn_speed_minus;
    uint8_t btn_vascula;
    uint8_t btn_shutdown;
    uint8_t mode;
    uint8_t raw_buttons;
    uint8_t valid;          /* 1 if data valid */
} JOY_Data_t;
/* Joystick data to SEND (in master mode) */
extern JOY_Data_t joy_data;
/* ECU state structure */



typedef struct __attribute__((packed)) {
    uint8_t header;     /* Always 0x4A */
    uint8_t buttons;    /* Button flags */
    uint8_t mode;       /* Operating mode */
    uint8_t joy_x;      /* X axis value */
    uint8_t joy_y;      /* Y axis value */
    uint8_t checksum;   /* (sum bytes 0-4) XOR 0xFF */
} JOY_Packet_t;
typedef struct __attribute__((packed)) {
    uint8_t header1;    /* Always 0xFE */
    uint8_t header2;    /* Always 0x54 */
    uint8_t status1;    /* System status flags */
    uint8_t status2;    /* Usually 0xA0 */
    uint8_t battery;    /* Battery level / error code */
    uint8_t speed_cmd;  /* Speed setting + command */
    uint8_t checksum;   /* (sum bytes 1-5) XOR 0xFF */
} ECU_Packet_t;








typedef enum {
    VR2_MASTER_WAIT_IDLE        = 0x00U,
    VR2_MASTER_IDLE             = 0x01U,
    VR2_MASTER_WAKE_PULSE       = 0x02U,  /* Sending wake pulse */
    VR2_MASTER_SEND_STARTUP     = 0x03U,  /* Sending 53 AC x5 */
    VR2_MASTER_INIT             = 0x04U,
    VR2_MASTER_RUNNING          = 0x05U,  /* Normal operation - sending JOY */
    VR2_MASTER_START_SHUTDOWN   = 0x06U,  
    VR2_MASTER_PROCESS_SHUTDOWN = 0x07U,



    VR2_MASTER_WAIT_ECU_STANDBY = 0x02U,  /* Waiting for bus LOW (ECU off) */
    VR2_MASTER_WAIT_STARTUP_TX  = 0x04U,  /* Waiting for startup TX complete */
    VR2_MASTER_WAIT_HANDSHAKE_TX= 0x05U,  /* Waiting for handshake TX complete */
    VR2_MASTER_WAIT_ECU_INIT    = 0x06U,  /* Sending init packets */
    VR2_MASTER_WAIT_ECU_INIT_RX = 0x07U,  /* Waiting for ECU init response */
    VR2_MASTER_WAIT_JOY_TX      = 0x09U,  /* Waiting for JOY TX complete */
    VR2_MASTER_WAIT_ECU_FRAME   = 0x0AU,  /* Waiting for ECU response */
    VR2_MASTER_SHUTDOWN         = 0x0BU,  /* ECU shutdown detected */
} VR2_MasterState;



typedef enum {
    VR2_CMD_NONE = 0,
    VR2_CMD_START,          // avvia protocollo
    VR2_CMD_SHUTDOWN,       // shutdown
    VR2_CMD_SPEED_PLUS,
    VR2_CMD_SPEED_MINUS,
    VR2_CMD_SET_JOY,        // aggiorna X/Y
} vr2_cmd_id_t;

typedef struct {
    vr2_cmd_id_t cmd;
    int8_t x;
    int8_t y;
    uint8_t buttons;
} vr2_command_t;




typedef enum {
    VR2_ECU_IDLE = 0,          
    VR2_ECU_INIT,       
    VR2_ECU_RUN,       
    VR2_ECU_ERROR,       
    VR2_ECU_SHUTDOWN,       
} vr2_ecu_state_t;

typedef struct {
    vr2_ecu_state_t state;
    uint8_t speed;
    uint8_t battery;
} vr2_state_t;
extern QueueHandle_t vr2_state_queue;

esp_err_t vr2_post_state(vr2_state_t state);



uint8_t vr2_calc_checksum_joy(const uint8_t *data);
uint8_t vr2_calc_checksum_ecu(const uint8_t *data);
uint8_t vr2_verify_checksum_joy(const JOY_Packet_t *packet);
uint8_t vr2_parse_ecu_packet(const uint8_t *buffer, ECU_Packet_t *packet);


/*============================================================================
 * PROTOCOL CONSTANTS
 *============================================================================*/

/* Frame Headers */
#define JOY_HEADER              0x4A
#define ECU_HEADER1             0xFE
#define ECU_HEADER2             0x54
#define STARTUP_BYTE1           0x53
#define STARTUP_BYTE2           0xAC
#define JOY_STARTUP_RESP1       0x73
#define JOY_STARTUP_RESP2       0x12

/* Frame Sizes */
#define JOY_PACKET_SIZE         6
#define ECU_PACKET_SIZE         7
#define STARTUP_PATTERN_SIZE    2  /* 5x [53 AC] */

/* Joy Buttons (Byte 1) */
#define JOY_BTN_NONE            0x00
#define JOY_BTN_SPEED_MINUS     0x02
#define JOY_BTN_SPEED_PLUS      0x04
#define JOY_BTN_VASCULA         0x08  /* Tilt mode */
#define JOY_BTN_SHUTDOWN        0x40  /* Shutdown confirm (hold) */
#define JOY_BTN_SHUTDOWN_START  0xC0  /* Shutdown start (bit7+bit6) */

/* Joy Mode (Byte 2) */
#define JOY_MODE_INIT_FIRST     0x81  /* First init message */
#define JOY_MODE_INIT_ZERO      0x00  /* Init variant */
#define JOY_MODE_OPERATIVE      0xA0  /* Normal operation */
#define JOY_MODE_ACK            0xA2  /* Acknowledgement */



/* ECU Status1 (Byte 2) */
#define ECU_STATUS1_NORMAL      0x00
#define ECU_STATUS1_SHUTDOWN    0x10
#define ECU_STATUS1_VASCULA     0x20
#define ECU_STATUS1_INIT        0x80
#define ECU_STATUS1_OFF         0x90
#define ECU_STATUS1_ERR         0x80

/* ECU Status2 (Byte 3) - usually constant */
#define ECU_STATUS2_OPERATIVE   0xA0
#define ECU_STATUS2_ACK         0xA2
#define ECU_STATUS2_NACK        0xA3  /* Acknowledgement */

/* ECU Battery Level (Byte 4) */
#define ECU_BATT_1LED           0x12  /* 21.8V */
#define ECU_BATT_2LED           0x22  /* 23.7V */
#define ECU_BATT_3LED           0x31  /* 24.0V - red */
#define ECU_BATT_4LED           0x41  /* 24.4V - orange */
#define ECU_BATT_5LED           0x51  /* 24.6V */
#define ECU_BATT_6LED           0x61  /* 24.8V */
#define ECU_BATT_7LED           0x71  /* 25.0V */
#define ECU_BATT_8LED           0x81  /* 25.1V - green */
#define ECU_BATT_9LED           0x91  /* 25.4V - green */
#define ECU_BATT_10LED          0xA1  /* 25.4V+ - green */

/* ECU Error Codes (in Battery byte position) */
#define ECU_ERR_MOTOR_LEFT      0x23
#define ECU_ERR_MOTOR_RIGHT     0x43
#define ECU_ERR_BRAKE           0x93

/* ECU Speed Command (Byte 5) */
#define ECU_CMD_SHUTDOWN        0x10
#define ECU_CMD_INIT            0x11
#define ECU_CMD_SPEED1          0x21  /* Slowest */
#define ECU_CMD_SPEED2          0x41
#define ECU_CMD_SPEED3          0x61
#define ECU_CMD_SPEED4          0x81
#define ECU_CMD_SPEED5          0xA1  /* Fastest */

/* Joystick center values */
#define JOY_CENTER_X            0x00
#define JOY_CENTER_Y            0x00
#define JOY_DEADZONE            0x10

/* Timing constants (in microseconds/milliseconds) */
#define INTER_FRAME_DELAY_US    350
#define STARTUP_PULSE_MS        108
#define FRAME_INTERVAL_MS       6  /* ~5.8ms between frames */
void vr2_emulator_start(void);
esp_err_t vr2_post_command(vr2_command_t cmd);



void vr2_joy_master_init(void);


VR2_MasterState vr2_joy_master_run(vr2_command_t cmd);




#endif