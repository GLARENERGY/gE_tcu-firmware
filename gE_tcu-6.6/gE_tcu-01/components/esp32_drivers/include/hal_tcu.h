#pragma once
/* ESP32-C6 implementation of tcu_board_hw_interface.h.
 * Call hal_tcu_attach() once, before hal_rtc_init() / hal_inclinometer_read(). */
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "pcf2131.h"

void hal_tcu_attach(i2c_master_bus_handle_t i2c_bus, QueueHandle_t tilt_queue);

/* Used by rtc_service */
pcf2131_t *hal_tcu_get_rtc(void);                 /* NULL if the RTC was not found */
void       hal_tcu_set_time_trusted(bool trusted);
bool       hal_time_is_trusted(void);   /* implemented in hal_tcu.c */
