#pragma once
/*
 * Wind speed from the RS485 Modbus-RTU anemometer (ProRange SN-3000-FSJT-N01, "Model 485").
 *
 *   ESP32-C6 UART1 TX/RX  <->  RS485 module (auto-direction, TX/RX only)  <->  anemometer A/B
 *
 * Sensor protocol (from the data sheet): 4800 baud, 8N1, default address 1.
 * The ESP32 sends the Modbus query  01 03 00 00 00 01 84 0A  (read holding register 0) and the
 * sensor answers  01 03 02 HH LL CRC_L CRC_H  where HHLL = wind speed x 10 in m/s
 * (e.g. 0x0056 = 86 = 8.6 m/s). Range 0..70 m/s, resolution 0.1 m/s.
 *
 * Pins and serial settings come from config.yaml (AppConfig.h):
 *   ANEMOMETER_TX_PIN, ANEMOMETER_RX_PIN        (required)
 *   ANEMOMETER_BAUD (4800), ANEMOMETER_ADDR (1) (optional)
 */
#include <stdbool.h>
#include "esp_err.h"

/* Starts the polling task (about once a second). */
esp_err_t anemometer_start(void);

/* Latest wind speed in m/s. Returns false when there is no valid reading, i.e. the sensor has
 * not answered correctly for a few seconds (unplugged, A/B swapped, wrong baud/address...). */
bool anemometer_get(float *speed_ms);
