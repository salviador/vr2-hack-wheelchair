idf_component_register(SRCS  "app_main.c"
                            "vr2_pgdrives.c" "bleService.c" "switch.c" "led.c"
                       REQUIRES nvs_flash bt esp_driver_uart esp_driver_gpio esp_timer
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "")
