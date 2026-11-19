idf_component_register(SRCS "status_led.c"
                       INCLUDE_DIRS "include"
                       REQUIRES board_hal tracker_core esp_driver_rmt freertos esp_timer)
