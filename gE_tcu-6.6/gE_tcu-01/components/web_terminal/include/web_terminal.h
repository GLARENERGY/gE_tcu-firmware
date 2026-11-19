#pragma once
/* Serial-monitor mirror for the web UI.
 *   web_terminal_init()      - call FIRST in app_main: captures everything printed through ESP_LOGx
 *                              (still forwarded to the UART/USB console) into a RAM ring buffer.
 *   web_terminal_register()  - registers GET /terminal (page) and GET /terminal/log (polled text) on the HTTP server.
 *                              activity_cb (optional) is called on every poll so the caller can tell a browser is connected. */
#include "esp_http_server.h"

void web_terminal_init(void);
void web_terminal_register(httpd_handle_t server, void (*activity_cb)(void));
