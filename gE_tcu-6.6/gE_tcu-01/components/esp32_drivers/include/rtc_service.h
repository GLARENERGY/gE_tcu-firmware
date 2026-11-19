#pragma once
/*
 * RTC service: PCF2131 minute alarm -> INTA -> ISR (notify only) -> RTC task.
 *
 *  INTA falling edge --ISR--> notify rtc task --> read RTC time
 *                                              --> arm alarm for the next full minute, clear AF
 *                                              --> inclinometer_trigger()
 *                                              --> callback(minute_epoch)
 */
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

typedef void (*rtc_minute_cb_t)(time_t minute_epoch, bool time_trusted);

/* Needs hal_tcu_attach() + hal_rtc_init() first. cb runs in the RTC task (not in the ISR). */
esp_err_t rtc_service_start(rtc_minute_cb_t cb);

/* Called by hal_set_time(): time was just synchronised -> re-arm for the next full minute. */
void rtc_service_on_time_set(void);
