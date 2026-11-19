#pragma once
/*
 * Inclinometer module (LIS2DW12 accelerometer) - single .h/.c pair.
 *
 *   LIS2DW12 @50 Hz -> block average -> tilt x/y/z (deg) -> tilt_msg_t queue (~5 Hz)
 *
 * x/y/z are per-axis tilt in DEGREES, same meaning as the old inclinometer_data_t,
 * so tracker_core's INCLINOMETER_AXIS / REVERSE / OFFSET logic keeps working.
 */
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

/* ---- Overridable from AppConfig.h / config.yaml ---- */
#ifndef ACCEL_I2C_ADDR
#define ACCEL_I2C_ADDR      0x18        /* 0x18 = SA0 low, 0x19 = SA0 high */
#endif
#ifndef ACCEL_I2C_HZ
#define ACCEL_I2C_HZ        100000
#endif
#ifndef ACCEL_AVG_SAMPLES
#define ACCEL_AVG_SAMPLES   10          /* 10 x 20 ms = one value per 200 ms */
#endif

/* ---- Message sent to the tracker ---- */
typedef struct {
    float    x, y, z;     /* tilt, degrees. z = 90 when the board lies flat */
    bool     valid;       /* false = sensor fault, do not drive the motor */
    uint32_t tick;        /* xTaskGetTickCount() when produced */
} tilt_msg_t;

static inline bool tilt_msg_is_fresh(const tilt_msg_t *m, uint32_t max_age_ms)
{
    return m->valid &&
           (uint32_t)(xTaskGetTickCount() - m->tick) <= pdMS_TO_TICKS(max_age_ms);
}

/* Starts the sampling task. `queue`: length 1, item size sizeof(tilt_msg_t). */
esp_err_t inclinometer_start(i2c_master_bus_handle_t bus, QueueHandle_t queue);

/* Restart the averaging window so the next published value is a completely fresh measurement.
 * Safe to call from any task; no-op if the task is not running (e.g. master byte 0). */
void inclinometer_trigger(void);
