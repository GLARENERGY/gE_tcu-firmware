#pragma once
#include <stdbool.h>
/* Wi-Fi AP+STA ("Glarenergy-TCU"), mDNS ("glarenergy"), HTTP server, SNTP, SPIFFS restart log.
 * Needs: NVS initialised, HAL initialised, global `tracker` created. */
void web_ui_start(void);

/* Appends a line to /storage/restarts.txt once system time is valid (no-op if track_restart is off).
 * Call periodically until it has logged. */
void log_restart(void);

/* true while a browser has polled /status or /terminal within the last ~5 s (used by the status LED). */
bool web_ui_client_connected(void);
