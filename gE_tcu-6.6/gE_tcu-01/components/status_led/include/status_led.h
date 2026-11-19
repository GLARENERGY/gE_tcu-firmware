#pragma once
/* Built-in WS2812 RGB LED status indicator (ESP32-C6-DevKitM-1: GPIO 8, set RGB_LED_PIN in config.yaml).
 *
 *  Normal AUTO, website not connected : light green   (1/3 idle, 2/3 while the motor moves)
 *  Normal AUTO, website connected     : purple        (1/3 idle, 2/3 while the motor moves)
 *  Inclinometer not detected          : red 1/3 (website not connected) / yellow 1/2 (connected)
 *  Clock not set + website not connected : blinking blue
 */
#include <stdbool.h>
#include "esp_err.h"
#include "tracker_core.h"

/* Returns true while a browser is actively talking to the TCU (e.g. polled within the last few seconds). */
typedef bool (*status_led_web_connected_fn)(void);

esp_err_t status_led_start(tracker_handle_t tracker, status_led_web_connected_fn web_connected);
