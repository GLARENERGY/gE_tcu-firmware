idf_component_register(SRCS "anemometer.c"
                       INCLUDE_DIRS "include"
                       REQUIRES board_hal esp_driver_uart freertos)
