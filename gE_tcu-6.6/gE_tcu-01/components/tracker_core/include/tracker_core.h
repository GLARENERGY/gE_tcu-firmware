#ifndef TRACKER_CORE_H
#define TRACKER_CORE_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "tcu_board_hw_interface.h"
#include "solar_math.h"
#include "cJSON.h"

typedef enum {
    TRACKER_STATE_OFF,
    TRACKER_STATE_TRACKING,
    TRACKER_STATE_MANUAL,
    TRACKER_STATE_NIGHT_RESET,
    TRACKER_STATE_STOW,
    TRACKER_STATE_STOP
} tracker_state_t;

typedef struct tracker_s *tracker_handle_t;

// --- API ---
tracker_handle_t tracker_init(void);
void tracker_update(tracker_handle_t tracker);
// Called by the RTC task every minute (minute-aligned RTC time). Starts a move when a new table slot begins.
void tracker_on_minute(tracker_handle_t tracker, time_t minute_epoch);

// Thread-safe snapshot for external reporting (web/Modbus). current_tilt/target_tilt in degrees.
void tracker_get_snapshot(tracker_handle_t tracker, float *current_tilt, float *target_tilt,
                           tracker_state_t *state, tracker_mode_t *effective_mode,
                           bool *motor_fault, bool *inclinometer_ok);

// Configuration & Overrides
void tracker_set_mode(tracker_handle_t tracker, tracker_mode_t mode);
void tracker_set_manual_target(tracker_handle_t tracker, float angle);
void tracker_set_web_buttons(tracker_handle_t tracker, bool up, bool down);
// Change panel width / row pitch (metres). Saved to NVS, day table is recalculated and the current slot is re-applied.
// Returns false (nothing changed) if the values are invalid.
bool tracker_set_geometry(tracker_handle_t tracker, float panel_width, float pitch_distance);

// Monitoring
tracker_state_t tracker_get_state(tracker_handle_t tracker);
cJSON* tracker_get_status_json(tracker_handle_t tracker);

#endif // TRACKER_CORE_H
