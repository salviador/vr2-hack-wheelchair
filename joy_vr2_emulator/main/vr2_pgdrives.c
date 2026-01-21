#include "vr2_pgdrives.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "VR2";

static TaskHandle_t s_vr2_task = NULL;
volatile VR2_MasterState vr2_master_state = VR2_MASTER_IDLE;
static QueueHandle_t vr2_cmd_queue;

static uint8_t master_rx_buffer[JOY_PACKET_SIZE + ECU_PACKET_SIZE + 2];  /* Extra bytes for safety */
static uint8_t master_tx_buffer[STARTUP_PATTERN_SIZE + 4];  /* Max size for startup */
static uint32_t master_timestamp = 0;

QueueHandle_t vr2_state_queue = NULL;


// millisecondi "real-time" (meglio di xTaskGetTickCount per finestre piccole)
uint32_t vr2_millis(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}


/*============================================================================
 * UART HANDLING
 *============================================================================*/
/*============================================================================
 * UART HANDLING
 *============================================================================*/
/*============================================================================
 * UART HANDLING
 *============================================================================*/
/*============================================================================
 * UART HANDLING
 *============================================================================*/
/*============================================================================
 * UART HANDLING
 *============================================================================*/

esp_err_t vr2_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = VR2_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,      // 8E1
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(VR2_UART_NUM, VR2_RX_BUF_SIZE, VR2_TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(VR2_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(VR2_UART_NUM, VR2_UART_TX_PIN, VR2_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Facoltativo: pulisci input
    uart_flush_input(VR2_UART_NUM);

    ESP_LOGI(TAG, "UART1 ready: %d 8E1, TX=%d RX=%d",
             VR2_UART_BAUDRATE, (int)VR2_UART_TX_PIN, (int)VR2_UART_RX_PIN);

    return ESP_OK;
}

int vr2_uart_write(const uint8_t *data, size_t len)
{
    return uart_write_bytes(VR2_UART_NUM, (const char *)data, len);
}

int vr2_uart_read(uint8_t *data, size_t maxlen, uint32_t timeout_ms)
{
    return uart_read_bytes(VR2_UART_NUM, data, maxlen, pdMS_TO_TICKS(timeout_ms));
}


/*============================================================================
 * GPIO CONTROL
 *============================================================================*/
 
/*============================================================================
 * GPIO CONTROL
 *============================================================================*/
 
/*============================================================================
 * GPIO CONTROL
 *============================================================================*/
 
/*============================================================================
 * GPIO CONTROL
 *============================================================================*/
void vr2_init_gpio_control(void)
{
    gpio_config_t io_conf = {0};

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;

    io_conf.pin_bit_mask =
        (1ULL << VR2_UART_EN_TX) |
        (1ULL << VR2_PUL_UP_EN)  |
        (1ULL << VR2_PUL_DW_EN)  |
        (1ULL << VR2_RELE_EN);

    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Stato iniziale sicuro (consigliato)
    gpio_set_level(VR2_UART_EN_TX, 1);
    gpio_set_level(VR2_PUL_UP_EN,  1);
    gpio_set_level(VR2_PUL_DW_EN,  1);
    gpio_set_level(VR2_RELE_EN,    0);
}


void vr2_set_rx_gpio_input(void)
{
    // scollega RX dalla UART
    ESP_ERROR_CHECK(uart_set_pin(
        VR2_UART_NUM,
        VR2_UART_TX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << VR2_UART_RX_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io));
}

uint8_t vr2_get_rx_gpio(void)
{
    return gpio_get_level(VR2_UART_RX_PIN);
}

void vr2_set_rx_uart(void)
{
    // rimappa RX alla UART
    ESP_ERROR_CHECK(uart_set_pin(
        VR2_UART_NUM,
        VR2_UART_TX_PIN,
        VR2_UART_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));
}



void vre_joy_pull_up(void)
{
    gpio_set_level(VR2_PUL_UP_EN,  0);
    gpio_set_level(VR2_PUL_DW_EN,  1);
}

void vre_joy_pull_down(void)
{
    gpio_set_level(VR2_PUL_UP_EN,  1);
    gpio_set_level(VR2_PUL_DW_EN,  0);
}

void vre_joy_pull_no(void)
{
    gpio_set_level(VR2_PUL_UP_EN,  1);
    gpio_set_level(VR2_PUL_DW_EN,  1);
}

void vre_joy_en_tx(void)
{
    gpio_set_level(VR2_UART_EN_TX, 0);
}

void vre_joy_dis_tx(void)
{
    gpio_set_level(VR2_UART_EN_TX, 1);
}


//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
uint8_t vr2_calc_checksum_joy(const uint8_t *data)
{
    uint8_t sum = data[0] + data[1] + data[2] + data[3] + data[4];
    return sum ^ 0xFF;
}

uint8_t vr2_calc_checksum_ecu(const uint8_t *data)
{
    uint8_t sum = data[1] + data[2] + data[3] + data[4] + data[5];
    return sum ^ 0xFF;
}

uint8_t vr2_verify_checksum_joy(const JOY_Packet_t *packet)
{
    const uint8_t *data = (const uint8_t *)packet;
    uint8_t calc = vr2_calc_checksum_joy(data);
    return (calc == packet->checksum) ? 1 : 0;
}


uint8_t vr2_parse_ecu_packet(const uint8_t *buffer, ECU_Packet_t *packet)
{
    memcpy(packet, buffer, ECU_PACKET_SIZE);

    /* Verify headers */
    if (packet->header1 != ECU_HEADER1 || packet->header2 != ECU_HEADER2) {
        return 0;
    }

    /* Verify checksum */
    uint8_t calc = vr2_calc_checksum_ecu((const uint8_t *)packet);
    if (calc != packet->checksum) {
        return 0;
    }

    return 1;
}



//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------
/*============================================================================
  VR2 MASTER PROTOCOL
 ============================================================================*/

 
void vr2_joy_master_init(void)
{
    vre_joy_dis_tx();
    vre_joy_pull_down();
    vr2_master_state = VR2_MASTER_WAIT_IDLE;
    master_timestamp = vr2_millis();
    xQueueReset(vr2_cmd_queue);
}




VR2_MasterState vr2_joy_master_run(vr2_command_t cmd)
{
    int rx_len;
    vr2_state_t ecustate = {0};

    JOY_Packet_t joy_packet = {0};
    ECU_Packet_t ecu_packet = {0};

    switch (vr2_master_state)
    {
        case VR2_MASTER_WAIT_IDLE:
        {
            ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_WAIT_IDLE");

            ecustate.state = VR2_ECU_IDLE;
            ecustate.battery = 0;
            ecustate.speed = 0;
            vr2_post_state(ecustate);

            vre_joy_pull_down();
            vTaskDelay(pdMS_TO_TICKS(500));
            //aspetta bus in IDLE
            vre_joy_dis_tx();
            vre_joy_pull_no();
            vr2_set_rx_gpio_input();
            uint32_t t_last_low = vr2_millis();
            while (1) {
                if (vr2_get_rx_gpio() == 1) {
                    // bus HIGH -> reset timer
                    t_last_low = vr2_millis();
                }

                if ((vr2_millis() - t_last_low) >= 500) {
                    break; // bus HIGH stabile abbastanza
                }
                // evita busy-wait: 1 tick (tipicamente 1ms se config standard)
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            xQueueReset(vr2_cmd_queue);
            vr2_master_state = VR2_MASTER_IDLE;
        }
        break;

        case VR2_MASTER_IDLE:
            if(cmd.cmd == VR2_CMD_START){
                ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_IDLE, **START**");
                ecustate.state = VR2_ECU_INIT;
                ecustate.battery = 0;
                ecustate.speed = 0;
                vr2_post_state(ecustate);

                joy_packet.joy_x = 0x00;
                joy_packet.joy_y = 0x00;
                vr2_master_state = VR2_MASTER_WAKE_PULSE;
                vTaskDelay(pdMS_TO_TICKS(250));
            }
        break;

        case VR2_MASTER_WAKE_PULSE:
        {
            ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_WAKE_PULSE");
            
            vre_joy_pull_no();
            esp_rom_delay_us(10);
            vre_joy_en_tx();
            
            gpio_config_t io_conf = {0};
            io_conf.pin_bit_mask = (1ULL << VR2_UART_TX_PIN);
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.intr_type = GPIO_INTR_DISABLE;
            ESP_ERROR_CHECK(gpio_config(&io_conf));
            
            gpio_set_level(VR2_UART_TX_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(108));
            gpio_set_level(VR2_UART_TX_PIN, 1);
            vre_joy_dis_tx();
            
            // NON riconfigurare ancora l'UART qui - lascia il TX come GPIO
            // Lo farai in SEND_STARTUP
            
            vre_joy_pull_up();
            vTaskDelay(pdMS_TO_TICKS(50));
            
            vr2_master_state = VR2_MASTER_SEND_STARTUP;
            master_timestamp = vr2_millis();
        }
        break;

        case VR2_MASTER_SEND_STARTUP:
        {
            uint8_t pattern[2] = { 0x53, 0xAC };
            
            ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_SEND_STARTUP");

            // ORA riconfigura l'UART (una sola volta)
            ESP_ERROR_CHECK(uart_set_pin(
                VR2_UART_NUM,
                VR2_UART_TX_PIN,
                VR2_UART_RX_PIN,
                UART_PIN_NO_CHANGE,
                UART_PIN_NO_CHANGE
            ));
            
            uart_flush(VR2_UART_NUM);
            uart_flush_input(VR2_UART_NUM);
            vre_joy_pull_up();
            
            for (int i = 0; i < 50; i++)
            {
                uart_flush(VR2_UART_NUM);
                uart_flush_input(VR2_UART_NUM);
                
                // Svuota completamente il buffer
                uint8_t dummy;
                while (uart_read_bytes(VR2_UART_NUM, &dummy, 1, 0) > 0);
                
                memset(master_rx_buffer, 0, sizeof(master_rx_buffer));
                
                // Trasmetti
                vre_joy_en_tx();
                int written = uart_write_bytes(VR2_UART_NUM, (const char*)pattern, 2);
                uart_wait_tx_done(VR2_UART_NUM, pdMS_TO_TICKS(10));
                vre_joy_dis_tx();
                vre_joy_pull_up();
                
                // Attendi risposta
                vTaskDelay(pdMS_TO_TICKS(10));  // Aumentato da 7 a 10ms
                
                // Leggi risposta
                int rx_len = uart_read_bytes(VR2_UART_NUM, 
                                            master_rx_buffer, 
                                            10,  // Buffer più grande
                                            pdMS_TO_TICKS(5));
                
                // LOG per debug: mostra cosa hai ricevuto
                if (rx_len > 0) {
                    /*
                    ESP_LOGI(TAG, "RX (%d bytes): %02X %02X %02X %02X %02X %02X", 
                            rx_len,
                            rx_len > 0 ? master_rx_buffer[0] : 0,
                            rx_len > 1 ? master_rx_buffer[1] : 0,
                            rx_len > 2 ? master_rx_buffer[2] : 0,
                            rx_len > 3 ? master_rx_buffer[3] : 0,
                            rx_len > 4 ? master_rx_buffer[4] : 0,
                            rx_len > 5 ? master_rx_buffer[5] : 0);
                    */
                }
                
                // Cerca SOLO 73 12 (non 73 12 81 F9)
                uint8_t handshake_found = 0;
                if (rx_len >= 2) {
                    for (int j = 0; j <= (rx_len - 2); j++) {
                        if (master_rx_buffer[j] == 0x73 && master_rx_buffer[j+1] == 0x12) {
                            handshake_found = 1;
                            ESP_LOGI(TAG, "Handshake found at position %d!", j);
                            break;
                        }
                    }
                }
                
                if (handshake_found) {
                    vr2_master_state = VR2_MASTER_INIT;
                    return vr2_master_state;
                }
                
                vTaskDelay(pdMS_TO_TICKS(5));  // Pausa tra tentativi
            }
            
            ESP_LOGW(TAG, "Handshake not found after 10 attempts, restarting...");
            vr2_joy_master_init();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        break;

        case VR2_MASTER_INIT:
        {
            ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_INIT");

            // Preparazione UART
            uart_flush(VR2_UART_NUM);
            uart_flush_input(VR2_UART_NUM);
            
            
            // Costruisci pacchetto INIT
            uint8_t *data = (uint8_t *)&joy_packet;
            joy_packet.header = JOY_HEADER;           // 0x4A
            joy_packet.buttons = JOY_BTN_NONE;        // 0x00
            joy_packet.mode = JOY_MODE_INIT_FIRST;    // 0x81
            joy_packet.joy_x = 0x00;
            joy_packet.joy_y = 0x00;
            joy_packet.checksum = vr2_calc_checksum_joy(data);

            // LOG del pacchetto da inviare
            ESP_LOGI(TAG, "TX INIT: %02X %02X %02X %02X %02X %02X",
                    joy_packet.header, joy_packet.buttons, joy_packet.mode,
                    joy_packet.joy_x, joy_packet.joy_y, joy_packet.checksum);

            memcpy(master_tx_buffer, &joy_packet, JOY_PACKET_SIZE);
            
            // Attendi inter-frame delay
            esp_rom_delay_us(INTER_FRAME_DELAY_US);

            // Trasmetti
            // TX
            vre_joy_en_tx();
            uart_write_bytes(VR2_UART_NUM, (char*)master_tx_buffer, JOY_PACKET_SIZE);
            uart_wait_tx_done(VR2_UART_NUM, pdMS_TO_TICKS(10));
            vre_joy_dis_tx();
            vre_joy_pull_up();

            //uart_flush_input(VR2_UART_NUM);


            // ⚠️ IMPORTANTE: Aspetta che l'ECU elabori e risponda
            //vTaskDelay(pdMS_TO_TICKS(2));  // 5ms di attesa

            // Leggi risposta con timeout adeguato
            memset(master_rx_buffer, 0, sizeof(master_rx_buffer));
            int rx_len = uart_read_bytes(VR2_UART_NUM, 
                                        master_rx_buffer, 
                                        (JOY_PACKET_SIZE + ECU_PACKET_SIZE),
                                        pdMS_TO_TICKS(10));  // 10ms timeout
            /*
            // LOG della risposta ricevuta
            if (rx_len > 0) {
                ESP_LOGI(TAG, "RX INIT (%d bytes): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", 
                        rx_len,
                        rx_len > 0 ? master_rx_buffer[0] : 0,
                        rx_len > 1 ? master_rx_buffer[1] : 0,
                        rx_len > 2 ? master_rx_buffer[2] : 0,
                        rx_len > 3 ? master_rx_buffer[3] : 0,
                        rx_len > 4 ? master_rx_buffer[4] : 0,
                        rx_len > 5 ? master_rx_buffer[5] : 0,
                        rx_len > 6 ? master_rx_buffer[6] : 0,
                        rx_len > 7 ? master_rx_buffer[7] : 0,
                        rx_len > 8 ? master_rx_buffer[8] : 0,
                        rx_len > 9 ? master_rx_buffer[9] : 0,
                        rx_len > 10 ? master_rx_buffer[10] : 0,
                        rx_len > 11 ? master_rx_buffer[11] : 0,
                        rx_len > 12 ? master_rx_buffer[12] : 0,
                        rx_len > 13 ? master_rx_buffer[13] : 0);
            } else {
                ESP_LOGW(TAG, "RX INIT: No data received!");
            }*/

            // Verifica lunghezza
            if (rx_len < (JOY_PACKET_SIZE + ECU_PACKET_SIZE)) {
                ESP_LOGW(TAG, "ECU INIT ERROR: Received only %d bytes, expected %d", 
                        rx_len, ECU_PACKET_SIZE);
                vr2_joy_master_init();
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }

            // Parse e verifica
            if (vr2_parse_ecu_packet(&master_rx_buffer[JOY_PACKET_SIZE], &ecu_packet)) {
                ESP_LOGI(TAG, "ECU packet valid - status1=0x%02X, status2=0x%02X, battery=0x%02X",
                        ecu_packet.status1, ecu_packet.status2, ecu_packet.battery);

                // Verifica status
                if (ecu_packet.status1 == ECU_STATUS1_INIT) {  // 0x80
                    ESP_LOGI(TAG, "ECU INIT OK -> RUNNING");
                    vr2_master_state = VR2_MASTER_RUNNING;
                    //vTaskDelay(pdMS_TO_TICKS(3));
                } else {
                    ESP_LOGW(TAG, "ECU INIT ERROR: Unexpected status1=0x%02X (expected 0x80)", 
                            ecu_packet.status1);
                    vr2_joy_master_init();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            } else {
                ESP_LOGW(TAG, "ECU INIT ERROR: Invalid packet (header or checksum)");
                vr2_joy_master_init();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        break;

        case VR2_MASTER_RUNNING:
        {
            vr2_master_state = VR2_MASTER_RUNNING;

            //ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_RUNNING");
            static int8_t joy_x_hold = 0;
            static int8_t joy_y_hold = 0;

            // aggiorna SOLO se arriva un nuovo comando joystick
            if (cmd.cmd == VR2_CMD_SET_JOY) {
                joy_x_hold = (int8_t)cmd.x;
                joy_y_hold = (int8_t)cmd.y;
            }

            uart_flush(VR2_UART_NUM);
            uart_flush_input(VR2_UART_NUM);
            uint8_t *data = (uint8_t *)&joy_packet;
            joy_packet.header = JOY_HEADER;  /* 0x4A */
            joy_packet.buttons = JOY_BTN_NONE;  /* 0x00 */
            joy_packet.mode = JOY_MODE_OPERATIVE;  /* 0xA0 */
            //joy_packet.joy_x = 0x00;
            //joy_packet.joy_y = 0x00;

            // SEMPRE invia l’ultimo valore (latch)
            joy_packet.joy_x = joy_x_hold;
            joy_packet.joy_y = joy_y_hold;


            if(ecu_packet.status2 == ECU_STATUS2_ACK){
                joy_packet.mode = JOY_MODE_ACK;
            }

            switch(cmd.cmd){
                case VR2_CMD_SHUTDOWN:
                    joy_packet.buttons = JOY_BTN_SHUTDOWN_START;  
                    vr2_master_state = VR2_MASTER_START_SHUTDOWN;
                break;

                case VR2_CMD_SPEED_PLUS:
                    joy_packet.buttons = JOY_BTN_SPEED_PLUS;
                break;

                case VR2_CMD_SPEED_MINUS:
                    joy_packet.buttons = JOY_BTN_SPEED_MINUS;
                break;
                case VR2_CMD_SET_JOY:
                    //joy_packet.joy_x = (int8_t)cmd.x;
                    //joy_packet.joy_y = (int8_t)cmd.y;                
                break;
                default:
                    //joy_packet.joy_x = 0x00;
                    //joy_packet.joy_y = 0x00;
                break;
            }
            joy_packet.checksum = vr2_calc_checksum_joy(data);
            memcpy(master_tx_buffer, &joy_packet, JOY_PACKET_SIZE);
            esp_rom_delay_us(INTER_FRAME_DELAY_US);

            /* 3) Trasmetti pattern */
           // Trasmetti
            // TX
            vre_joy_en_tx();
            uart_write_bytes(VR2_UART_NUM, (char*)master_tx_buffer, JOY_PACKET_SIZE);
            uart_wait_tx_done(VR2_UART_NUM, pdMS_TO_TICKS(10));
            vre_joy_dis_tx();
            vre_joy_pull_up();


            /* 5) Leggi i dati ricevuti */
            rx_len = uart_read_bytes(VR2_UART_NUM, 
                                        master_rx_buffer, 
                                        (JOY_PACKET_SIZE + ECU_PACKET_SIZE),
                                        pdMS_TO_TICKS(10));
            
            // Verifica lunghezza
            if (rx_len < (JOY_PACKET_SIZE + ECU_PACKET_SIZE)) {
                ESP_LOGW(TAG, "ECU ERROR: Received only %d bytes, expected %d", 
                        rx_len, ECU_PACKET_SIZE);
                vr2_joy_master_init();
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }

            if (vr2_parse_ecu_packet(&master_rx_buffer[JOY_PACKET_SIZE], &ecu_packet)) {  //Verifica cheksum


                /* Check if ECU is still in init mode */
                
                switch(ecu_packet.status1){
                    case ECU_STATUS1_INIT:

                        //INIT o ERROR ?????????????
                        if(ecu_packet.battery == 0x01){
                            //INIT
                        }else{
                            //ERROR
                            ecustate.battery = ecu_packet.battery;
                            ecustate.speed = ecu_packet.speed_cmd;
                            ecustate.state = VR2_ECU_ERROR;
                            vr2_post_state(ecustate);

                            ESP_LOGW(TAG, "ECU INIT ERROR!!! ...");                        
                            vr2_joy_master_init();  // Restart totale
                            vTaskDelay(pdMS_TO_TICKS(1000));
                        }
                    break;
                    case ECU_STATUS1_NORMAL:
                        ecustate.battery = ecu_packet.battery;
                        ecustate.speed = ecu_packet.speed_cmd;
                        ecustate.state = VR2_ECU_RUN;
                        vr2_post_state(ecustate);
                    break;
                    case ECU_STATUS1_OFF:

                    break;
                    default:
                        ESP_LOGW(TAG, "ECU INIT ERROR!!! ...");                        
                        vr2_joy_master_init();  // Restart totale
                        vTaskDelay(pdMS_TO_TICKS(1000));
                    break;
                }
                switch(ecu_packet.status2){
                    //ACK REQUEST
                    case JOY_MODE_ACK:

                    break;
                    case ECU_STATUS2_NACK:

                    break;
                    default:
                    break;
                }
                //vTaskDelay(pdMS_TO_TICKS(3));                    
            }else {
                /* Invalid packet, retry */
                ecustate.battery = ecu_packet.battery;
                ecustate.speed = ecu_packet.speed_cmd;
                ecustate.state = VR2_ECU_ERROR;
                vr2_post_state(ecustate);

                ESP_LOGW(TAG, "ECU INIT ERROR!!! ...");
                vr2_joy_master_init();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        break;


        case VR2_MASTER_START_SHUTDOWN:
        {
            ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_START_SHUTDOWN");

            ecustate.battery = 0;
            ecustate.speed = 0;
            ecustate.state = VR2_ECU_SHUTDOWN;
            vr2_post_state(ecustate);

            uart_flush(VR2_UART_NUM);
            uart_flush_input(VR2_UART_NUM);
            uint8_t *data = (uint8_t *)&joy_packet;

            joy_packet.header = JOY_HEADER;  /* 0x4A */
            joy_packet.buttons = JOY_BTN_SHUTDOWN_START;  
            joy_packet.mode = JOY_MODE_OPERATIVE;  /* 0xA0 */
            joy_packet.joy_x = 0x00;
            joy_packet.joy_y = 0x00;
            
            joy_packet.checksum = vr2_calc_checksum_joy(data);
            memcpy(master_tx_buffer, &joy_packet, JOY_PACKET_SIZE);
            esp_rom_delay_us(INTER_FRAME_DELAY_US);

            /* 3) Trasmetti pattern */
            vre_joy_en_tx();
            uart_write_bytes(VR2_UART_NUM, (char*)master_tx_buffer, JOY_PACKET_SIZE);
            uart_wait_tx_done(VR2_UART_NUM, pdMS_TO_TICKS(10));
            vre_joy_dis_tx();
            vre_joy_pull_up();


            /* 5) Leggi i dati ricevuti */
            rx_len = uart_read_bytes(VR2_UART_NUM, 
                                        master_rx_buffer, 
                                        (JOY_PACKET_SIZE + ECU_PACKET_SIZE),
                                        pdMS_TO_TICKS(10));
            

            if (vr2_parse_ecu_packet(&master_rx_buffer[JOY_PACKET_SIZE], &ecu_packet)) {  //Verifica cheksum
                vr2_master_state = VR2_MASTER_PROCESS_SHUTDOWN;
                //vTaskDelay(pdMS_TO_TICKS(3));                    
            }else {
                /* Invalid packet, retry */
                ESP_LOGW(TAG, "ECU INIT ERROR!!! ...");
                vr2_joy_master_init();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        break;

        case VR2_MASTER_PROCESS_SHUTDOWN:
        {
            ESP_LOGI(TAG, "[EMULATOR], VR2_MASTER_PROCESS_SHUTDOWN");

            xQueueReset(vr2_cmd_queue);

            uart_flush(VR2_UART_NUM);
            uart_flush_input(VR2_UART_NUM);
            uint8_t *data = (uint8_t *)&joy_packet;

            joy_packet.header = JOY_HEADER;  /* 0x4A */
            joy_packet.buttons = JOY_BTN_SHUTDOWN;  
            joy_packet.mode = JOY_MODE_OPERATIVE;  /* 0xA0 */
            joy_packet.joy_x = 0x00;
            joy_packet.joy_y = 0x00;
            
            joy_packet.checksum = vr2_calc_checksum_joy(data);
            memcpy(master_tx_buffer, &joy_packet, JOY_PACKET_SIZE);
            esp_rom_delay_us(INTER_FRAME_DELAY_US);

            /* 3) Trasmetti pattern */
            vre_joy_en_tx();
            uart_write_bytes(VR2_UART_NUM, (char*)master_tx_buffer, JOY_PACKET_SIZE);
            uart_wait_tx_done(VR2_UART_NUM, pdMS_TO_TICKS(10));
            vre_joy_dis_tx();
            vre_joy_pull_up();


            /* 5) Leggi i dati ricevuti */
            rx_len = uart_read_bytes(VR2_UART_NUM, 
                                        master_rx_buffer, 
                                        (JOY_PACKET_SIZE + ECU_PACKET_SIZE),
                                        pdMS_TO_TICKS(10));
            

            if (vr2_parse_ecu_packet(&master_rx_buffer[JOY_PACKET_SIZE], &ecu_packet)) {  //Verifica cheksum
                if(ecu_packet.status1 == ECU_STATUS1_OFF){
                    ESP_LOGW(TAG, "ECU SHUTDOWN!!! ...");
                    vr2_joy_master_init();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
                vTaskDelay(pdMS_TO_TICKS(3));                    
            }else {
                /* Invalid packet, retry */
                ESP_LOGW(TAG, "ECU INIT ERROR!!! ...");
                vr2_joy_master_init();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        break;

















        default:
            /* Invalid packet, retry */
            ESP_LOGW(TAG, "ECU INIT ERROR!!! ...");
            vr2_joy_master_init();
            vTaskDelay(pdMS_TO_TICKS(1000));
        break;
    }
    return vr2_master_state;
}




















/*============================================================================
 * TASK CONTROL
 *============================================================================*/
static void vr2_task(void *arg)
{
    (void)arg;

    configASSERT(vr2_cmd_queue);
    configASSERT(vr2_state_queue);

    vr2_command_t cmd = { .cmd = VR2_CMD_NONE };
    vr2_command_t tmp;

    //TickType_t last_wake = xTaskGetTickCount();

    vr2_init_gpio_control();
    vr2_uart_init();
    vr2_joy_master_init();

    while (1) {

        // Prendi l'ultimo comando disponibile (svuota la coda)
        while (xQueueReceive(vr2_cmd_queue, &tmp, 0) == pdTRUE) {
            cmd = tmp;
        }

        vr2_joy_master_run(cmd);

        // Se i comandi sono "one-shot", resetta dopo averli consumati
        cmd.cmd = VR2_CMD_NONE;

        //vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
        vTaskDelay(pdMS_TO_TICKS(5));

    }
}



void vr2_emulator_start(void)
{
    if (s_vr2_task) {
        return;
    }

    // Crea coda comandi PRIMA del task (così chi invia comandi non rischia NULL)
    vr2_cmd_queue = xQueueCreate(1, sizeof(vr2_command_t));
    configASSERT(vr2_cmd_queue);

    vr2_state_queue = xQueueCreate(1, sizeof(vr2_state_t));
    configASSERT(vr2_state_queue);

    // Task con priorità alta, ma non massima
    xTaskCreatePinnedToCore(
        vr2_task,
        "vr2_task",
        4096,
        NULL,
        10,
        &s_vr2_task,
        1   // di solito core 1 è ok (lascia core 0 più libero per Wi-Fi/BLE)
    );
}


esp_err_t vr2_post_command(vr2_command_t cmd)
{
    if (!vr2_cmd_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    // Non blocca: se la coda è piena, scarta
    /*
    if (xQueueSend(vr2_cmd_queue, &cmd, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }*/
    xQueueOverwrite( vr2_cmd_queue, &cmd );

    return ESP_OK;
}



esp_err_t vr2_post_state(vr2_state_t state)
{
    if (!vr2_state_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    xQueueOverwrite( vr2_state_queue, &state);
    return ESP_OK;
}




