idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_driver_i2c freertos
                                tracker_core solar_math board_hal esp32_drivers inclinometer anemometer
                                web_ui eth_w5500 modbus_tcp status_led web_terminal)
