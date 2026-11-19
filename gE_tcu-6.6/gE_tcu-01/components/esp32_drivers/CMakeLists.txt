idf_component_register(SRCS "hal_tcu.c" "rtc_service.c"
                       INCLUDE_DIRS "include"
                       REQUIRES board_hal inclinometer pcf2131 esp_driver_gpio esp_driver_i2c esp_timer freertos)
