#include "tracker_core.h"
#include "AppConfig.h"
#include "anemometer.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static SemaphoreHandle_t s_trk_mutex;   /* web handlers and tracker_task share the tracker */
#define TRK_LOCK()   do { if (s_trk_mutex) xSemaphoreTakeRecursive(s_trk_mutex, portMAX_DELAY); } while (0)
#define TRK_UNLOCK() do { if (s_trk_mutex) xSemaphoreGiveRecursive(s_trk_mutex); } while (0)


static const char *TAG = "TRACKER_CORE";

#ifndef TRACKER_INTERVAL_MINUTES
#define TRACKER_INTERVAL_MINUTES 30
#endif

#define TRACKER_INTERVAL_SECONDS (TRACKER_INTERVAL_MINUTES * 60)

/* Motor protection for a slot move (override in config.yaml) */
#ifndef MOTOR_MAX_RUN_S
#define MOTOR_MAX_RUN_S        300    /* one move may not run longer than this */
#endif
#ifndef MOTOR_NO_PROGRESS_S
#define MOTOR_NO_PROGRESS_S    30     /* error must shrink by MOTOR_MIN_PROGRESS_DEG within this time */
#endif
#ifndef MOTOR_MIN_PROGRESS_DEG
#define MOTOR_MIN_PROGRESS_DEG 0.3f
#endif
#define RTC_TICK_FALLBACK_MS   120000 /* no RTC tick for this long -> detect slot changes from system time */
#define TRACKER_TABLE_SIZE (1440 / TRACKER_INTERVAL_MINUTES)

/* Maintenance button target angle (deg) */
#ifndef MAINTENANCE_ANGLE
#define MAINTENANCE_ANGLE      45.0f
#endif
/* Wind stow: when wind >= WIND_STOW_SPEED_MS continuously for WIND_STOW_TRIGGER_MS the panel goes to
 * the stow angle and is locked there for WIND_STOW_HOLD_MS. Nothing (AUTO, web angle commands) can
 * move it during the hold; only a manual button press cancels it. When the hold ends the wind is
 * checked again: still >= the limit -> another hold, otherwise normal operation (auto / night)
 * resumes. Not applied at night (already 0 deg). */
#ifndef WIND_STOW_SPEED_MS
#define WIND_STOW_SPEED_MS     10.0f
#endif
#ifndef WIND_STOW_HOLD_MS
#define WIND_STOW_HOLD_MS      60000u
#endif
#ifndef WIND_STOW_TRIGGER_MS
#define WIND_STOW_TRIGGER_MS   5000u    /* wind must stay >= limit this long before stowing */
#endif
/* Serial-monitor target-angle logging threshold (deg) */
#define LOG_TARGET_STEP_DEG    0.05f

struct tracker_s {
    tracker_state_t state;
    tracker_mode_t physical_mode;
    tracker_mode_t effective_mode;
    tracker_mode_t web_mode;
    bool web_override;
    bool web_override_active_angle;
    float manual_target;
    bool manual_target_seeded;
    
    bool web_up;
    bool web_down;
    
    inclinometer_data_t last_inc;
    bool last_inc_ok;
    
    uint32_t loop_count;
    
    cJSON* cached_table_obj;
    float cached_table_values[TRACKER_TABLE_SIZE];
    time_t last_table_day;
    uint32_t last_table_calc;
    
    float table_min;
    float table_max;
    
    time_t cached_day_start_epoch;
    time_t cached_day_start_t;
    
    tracking_mode_t tracking_mode;
    bool correction_active;

    /* Slot scheduling: one move per table entry, started by the RTC minute tick */
    int   last_slot_idx;        // table slot already handled (-1 = none)
    bool  slot_move_active;     // motor is being driven to slot_target
    float slot_target;
    bool  motor_fault;          // wrong direction / stalled / timed out; cleared at next slot or mode change
    uint32_t last_tick_ms;      // when the last RTC minute tick arrived
    uint32_t move_start_ms;
    uint32_t progress_ms;
    float    best_err;
    tracker_state_t prev_state;

    bool  wind_stow;            // latched: wind above limit, panel held at stow angle
    bool  wind_suppress;        // stow was cancelled by a manual press: no new stow until wind drops
    uint32_t wind_stow_end_ms;  // hold ends at this hal_get_millis() time
    bool     wind_high_timing;  // wind >= limit, trigger timer running
    uint32_t wind_high_since_ms;// when the wind first reached the limit
    bool  log_target_valid;
    float log_target;           // last target angle printed to serial
    tracker_state_t log_state;
};

static const char* state_to_str(tracker_state_t s) {
    switch (s) {
        case TRACKER_STATE_OFF: return "OFF";
        case TRACKER_STATE_TRACKING: return "TRACKING";
        case TRACKER_STATE_MANUAL: return "MANUAL";
        case TRACKER_STATE_NIGHT_RESET: return "NIGHT_RESET";
        case TRACKER_STATE_STOW: return "STOW";
        case TRACKER_STATE_STOP: return "STOP";
        default: return "UNKNOWN";
    }
}

static void log_event(const char *msg) {
    ESP_LOGI(TAG, "Event: %s", msg);
}

static void ensure_table_cached(tracker_handle_t tracker, time_t day_start) {
    uint32_t now = hal_get_millis();
    if (!tracker->cached_table_obj || day_start != tracker->last_table_day || (now - tracker->last_table_calc > 900000)) {
        if (tracker->cached_table_obj) cJSON_Delete(tracker->cached_table_obj);
        
        tracker->cached_table_obj = cJSON_CreateArray();

        if (day_start < 1000000000) {
            for (int i = 0; i < TRACKER_TABLE_SIZE; i++) {
                tracker->cached_table_values[i] = 0.0f;
                cJSON_AddItemToArray(tracker->cached_table_obj, cJSON_CreateNumber(0.0));
            }
            tracker->table_min = 0.0f;
            tracker->table_max = 0.0f;
            tracker->tracking_mode = TRACKING_MODE_STANDARD;
            tracker->last_table_day = day_start;
            tracker->last_table_calc = now;
            return;
        }

        // Set overall tracking mode from compile-time configuration
#if SOLAR_BACKTRACKING_ENABLED
        if (fabsf(SOLAR_GROUND_SLOPE) < 0.01f) {
            tracker->tracking_mode = TRACKING_MODE_FLAT_BACKTRACKING;
        } else {
            tracker->tracking_mode = TRACKING_MODE_SLOPE_BACKTRACKING;
        }
#else
        tracker->tracking_mode = TRACKING_MODE_STANDARD;
#endif

        for (int i = 0; i < TRACKER_TABLE_SIZE; i++) {
            sun_position_t sun = solar_math_calculate_sun_position(day_start + i * TRACKER_INTERVAL_SECONDS);
            tracking_tilt_result_t tilt_result = solar_math_calculate_tracking_tilt_advanced(sun.elevation, sun.azimuth);
            float tilt_raw = tilt_result.tracking_tilt;
            float tilt_bounded = tilt_raw;
            if (tilt_bounded > SOLAR_MAX_TILT) tilt_bounded = SOLAR_MAX_TILT;
            if (tilt_bounded < -SOLAR_MAX_TILT) tilt_bounded = -SOLAR_MAX_TILT;

            tracker->cached_table_values[i] = tilt_bounded;

            float table_val = tilt_bounded;
#ifdef TABLE_UNBOUNDED
            table_val = tilt_raw;
#endif
            cJSON_AddItemToArray(tracker->cached_table_obj, cJSON_CreateNumber(roundf(table_val * 10.0f) / 10.0f));
        }
        float min_val = tracker->cached_table_values[0];
        float max_val = tracker->cached_table_values[0];
        for (int i = 1; i < TRACKER_TABLE_SIZE; i++) {
            float v = tracker->cached_table_values[i];
            if (v < min_val) min_val = v;
            if (v > max_val) max_val = v;
        }
        tracker->table_min = min_val;
        tracker->table_max = max_val;
        tracker->last_table_day = day_start;
        tracker->last_table_calc = now;
    }
}

static float clamp_to_table(tracker_handle_t tracker, float value) {
    if (value < tracker->table_min) return tracker->table_min;
    if (value > tracker->table_max) return tracker->table_max;
    return value;
}

tracker_handle_t tracker_init(void) {
    if (!s_trk_mutex) s_trk_mutex = xSemaphoreCreateRecursiveMutex();
    tracker_handle_t tracker = calloc(1, sizeof(struct tracker_s));
    tracker->state = TRACKER_STATE_MANUAL;
    tracker->physical_mode = hal_get_mode_switch();
    tracker->effective_mode = tracker->physical_mode;
    tracker->last_slot_idx = -1;
    tracker->prev_state = TRACKER_STATE_MANUAL;
    return tracker;
}

static float get_active_angle(inclinometer_data_t inc) {
#if INCLINOMETER_AXIS == 0
    return inc.x;
#elif INCLINOMETER_AXIS == 1
    return inc.y;
#elif INCLINOMETER_AXIS == 2
    return inc.z;
#else
    return inc.y;
#endif
}

static time_t local_day_start(time_t t, const solar_config_t *cfg) {
    struct tm dt_local;
    time_t local_t = t + (time_t)(cfg->timezone_offset * 3600);
    gmtime_r(&local_t, &dt_local);
    struct tm midnight = dt_local;
    midnight.tm_hour = 0; midnight.tm_min = 0; midnight.tm_sec = 0;
    return timegm(&midnight) - (time_t)(cfg->timezone_offset * 3600);
}

static void start_slot_move(tracker_handle_t tracker, int idx, float target, const char *why) {
    tracker->last_slot_idx = idx;
    tracker->slot_target = clamp_to_table(tracker, target);
    tracker->slot_move_active = true;
    tracker->motor_fault = false;
    tracker->move_start_ms = hal_get_millis();
    tracker->progress_ms = tracker->move_start_ms;
    tracker->best_err = 1e9f;
    ESP_LOGI(TAG, "Slot %d (%d min step) -> target %.2f deg [%s]", idx, TRACKER_INTERVAL_MINUTES,
             (double)tracker->slot_target, why);
}

/* Target angle currently being driven to; false when the motor is not being driven to an angle. */
static bool active_target(tracker_handle_t tracker, float *target) {
    switch (tracker->state) {
        case TRACKER_STATE_STOW:        *target = SOLAR_STOW_ANGLE; return true;
        case TRACKER_STATE_NIGHT_RESET: *target = SOLAR_PARK_ANGLE; return true;
        case TRACKER_STATE_TRACKING:
            if (tracker->slot_move_active) { *target = tracker->slot_target; return true; }
            return false;
        case TRACKER_STATE_MANUAL:
            if (tracker->web_override_active_angle) { *target = tracker->manual_target; return true; }
            return false;
        default: return false;
    }
}

/* Print every target-angle update to the serial monitor. */
static void log_angle_updates(tracker_handle_t tracker) {
    float tgt;
    bool has_tgt = active_target(tracker, &tgt);

    if (tracker->state != tracker->log_state) {
        ESP_LOGI(TAG, "STATE -> %s", state_to_str(tracker->state));
        tracker->log_state = tracker->state;
    }
    if (has_tgt && (!tracker->log_target_valid || fabsf(tgt - tracker->log_target) >= LOG_TARGET_STEP_DEG)) {
        ESP_LOGI(TAG, "TARGET ANGLE: %.2f deg [%s]", (double)tgt, state_to_str(tracker->state));
        tracker->log_target = tgt;
        tracker->log_target_valid = true;
    } else if (!has_tgt) {
        tracker->log_target_valid = false;
    }
}

/* Cancel an active stow hold (manual button press). Wind stays suppressed until wind drops. */
static void wind_stow_cancel(tracker_handle_t tracker, const char *why) {
    if (!tracker->wind_stow) return;
    tracker->wind_stow = false;
    tracker->wind_suppress = true;
    ESP_LOGW(TAG, "Wind stow hold cancelled (%s)", why);
}

static void tracker_update_impl(tracker_handle_t tracker) {
    time_t t = hal_get_time();
    tracker_mode_t physical_mode = hal_get_mode_switch();
    solar_config_t cfg;
    solar_math_get_config(&cfg);

    // Read inclinometer every 200ms (2 loops at 10Hz) to improve feedback response
    bool fresh_inc = false;
    if (tracker->loop_count % 2 == 0) {
        tracker->last_inc_ok = hal_inclinometer_read(&tracker->last_inc);
        fresh_inc = true;
    }

    tracker->effective_mode = tracker->web_override ? tracker->web_mode : physical_mode;
    hal_set_mode_indicator(tracker->effective_mode);

    // Cache day_start_local_epoch — only recompute when crossing local midnight
    time_t day_start_local_epoch;
    if (tracker->cached_day_start_epoch == 0 ||
        t - tracker->cached_day_start_epoch >= 86400 ||
        t < tracker->cached_day_start_epoch) {
        struct tm dt_local;
        time_t local_t = t + (time_t)(cfg.timezone_offset * 3600);
        gmtime_r(&local_t, &dt_local);
        struct tm midnight = dt_local;
        midnight.tm_hour = 0; midnight.tm_min = 0; midnight.tm_sec = 0;
        day_start_local_epoch = timegm(&midnight) - (time_t)(cfg.timezone_offset * 3600);
        tracker->cached_day_start_epoch = day_start_local_epoch;
        tracker->cached_day_start_t = t;
    } else {
        day_start_local_epoch = tracker->cached_day_start_epoch;
    }
    ensure_table_cached(tracker, day_start_local_epoch);

    // Table target for control — snap to current table entry
    float discrete_target = 0;
    float time_diff = (float)(t - day_start_local_epoch);
    float idx_f = time_diff / (float)TRACKER_INTERVAL_SECONDS;
    int idx = (int)idx_f;
    
    if (idx < 0) {
        discrete_target = tracker->cached_table_values[0];
    } else if (idx >= TRACKER_TABLE_SIZE) {
        discrete_target = tracker->cached_table_values[TRACKER_TABLE_SIZE - 1];
    } else {
        discrete_target = tracker->cached_table_values[idx];
    }

    int slot_idx = idx < 0 ? 0 : (idx >= TRACKER_TABLE_SIZE ? TRACKER_TABLE_SIZE - 1 : idx);

    if (physical_mode != tracker->physical_mode) {
        tracker->web_override = false;
        tracker->motor_fault = false;
        tracker->physical_mode = physical_mode;
        if (physical_mode == TRACKER_MODE_MANUAL) wind_stow_cancel(tracker, "manual switch");
        if (physical_mode == TRACKER_MODE_MANUAL && tracker->last_inc_ok) {
            tracker->manual_target = clamp_to_table(tracker, discrete_target);
            tracker->manual_target_seeded = true;
        }
        log_event(physical_mode == TRACKER_MODE_AUTO ? "Switch: AUTO" : "Switch: MANUAL");
    }

    // --- State Machine ---
    if (tracker->effective_mode == TRACKER_MODE_MANUAL) {
        tracker->state = TRACKER_STATE_MANUAL;
    } else if (fresh_inc) {
        sun_position_t sun = solar_math_calculate_sun_position(t);
        // Hysteresis: -0.5 to 0.0 elevation to avoid rapid toggling at horizon
        static bool was_night = false;
        float night_threshold = was_night ? 0.5f : -0.5f; 
        
        if (sun.elevation < night_threshold) {
            tracker->state = TRACKER_STATE_NIGHT_RESET;
            was_night = true;
        } else {
            tracker->state = TRACKER_STATE_TRACKING;
            was_night = false;
        }
    }

    // --- Wind stow: locked hold, see WIND_STOW_* above ---
    if (fresh_inc) {
        float wind = 0.0f;
        bool wind_ok = anemometer_get(&wind);
        uint32_t now_ms = hal_get_millis();
        bool manual_press = hal_is_up_button_pressed() || hal_is_down_button_pressed() ||
                            tracker->web_up || tracker->web_down;
        bool night = (tracker->state == TRACKER_STATE_NIGHT_RESET);
        if (!night && wind_ok && wind >= WIND_STOW_SPEED_MS) {   // sun check also covers MANUAL mode
            night = solar_math_calculate_sun_position(t).elevation < -0.5f;
        }

        if (tracker->wind_stow) {
            if (night) {
                tracker->wind_stow = false;                       // already parked at 0 deg
                ESP_LOGI(TAG, "Night: wind stow not needed");
            } else if (manual_press) {
                wind_stow_cancel(tracker, "manual button pressed");
            } else if ((int32_t)(now_ms - tracker->wind_stow_end_ms) >= 0 && wind_ok) {
                if (wind >= WIND_STOW_SPEED_MS) {
                    tracker->wind_stow_end_ms = now_ms + WIND_STOW_HOLD_MS;
                    ESP_LOGW(TAG, "WIND %.1f m/s still > %.1f m/s -> stow held another %u s",
                             (double)wind, (double)WIND_STOW_SPEED_MS, (unsigned)(WIND_STOW_HOLD_MS / 1000));
                } else {
                    tracker->wind_stow = false;
                    ESP_LOGI(TAG, "WIND %.1f m/s - stow hold finished, normal operation", (double)wind);
                }
            }
        } else {
            if (tracker->wind_suppress && wind_ok && wind < WIND_STOW_SPEED_MS) tracker->wind_suppress = false;

            if (!night && !tracker->wind_suppress && wind_ok && wind >= WIND_STOW_SPEED_MS) {
                if (!tracker->wind_high_timing) {
                    tracker->wind_high_timing = true;
                    tracker->wind_high_since_ms = now_ms;
                    ESP_LOGI(TAG, "WIND %.1f m/s >= %.1f m/s - confirming for %u s",
                             (double)wind, (double)WIND_STOW_SPEED_MS, (unsigned)(WIND_STOW_TRIGGER_MS / 1000));
                } else if ((uint32_t)(now_ms - tracker->wind_high_since_ms) >= WIND_STOW_TRIGGER_MS) {
                    tracker->wind_high_timing = false;
                    tracker->wind_stow = true;
                    tracker->wind_stow_end_ms = now_ms + WIND_STOW_HOLD_MS;
                    tracker->slot_move_active = false;
                    tracker->web_override_active_angle = false;
                    ESP_LOGW(TAG, "WIND >= %.1f m/s for %u s -> STOW (%.1f deg) held for %u s",
                             (double)WIND_STOW_SPEED_MS, (unsigned)(WIND_STOW_TRIGGER_MS / 1000),
                             (double)SOLAR_STOW_ANGLE, (unsigned)(WIND_STOW_HOLD_MS / 1000));
                }
            } else {
                tracker->wind_high_timing = false;   // wind dropped, night, suppressed or no reading: restart timer
            }
        }
    }
    if (tracker->wind_stow) tracker->state = TRACKER_STATE_STOW;

    switch (tracker->state) {
        case TRACKER_STATE_STOW:
            if (fresh_inc) {
                if (tracker->last_inc_ok) hal_motor_move_to_angle(SOLAR_STOW_ANGLE, tracker->last_inc);
                else                      hal_motor_stop();
            }
            break;
        case TRACKER_STATE_TRACKING:
            if (hal_time_is_trusted()) {
                if (tracker->prev_state != TRACKER_STATE_TRACKING) {
                    /* just entered tracking (boot, manual->auto, sunrise): catch up to the current slot */
                    start_slot_move(tracker, slot_idx, discrete_target, "entered tracking");
                } else if (slot_idx != tracker->last_slot_idx &&
                           (hal_get_millis() - tracker->last_tick_ms) > RTC_TICK_FALLBACK_MS) {
                    /* RTC minute tick is missing -> do not let the tracker freeze */
                    start_slot_move(tracker, slot_idx, discrete_target, "fallback: no RTC tick");
                }
            }
            if (fresh_inc) {
                if (!tracker->last_inc_ok) {
                    hal_motor_stop();                    // no valid angle: never drive blind (move resumes when it returns)
                } else if (tracker->slot_move_active && !tracker->motor_fault) {
                    float cur = get_active_angle(tracker->last_inc);
                    float err = fabsf(tracker->slot_target - cur);
                    uint32_t now_ms = hal_get_millis();
                    if (err < MOTOR_DEADBAND) {              // reached the SolTrack angle
                        hal_motor_stop();
                        tracker->slot_move_active = false;
                        ESP_LOGI(TAG, "Reached %.2f deg (target %.2f)", (double)cur, (double)tracker->slot_target);
                    } else {
                        if (err < tracker->best_err - MOTOR_MIN_PROGRESS_DEG) {   // getting closer
                            tracker->best_err = err;
                            tracker->progress_ms = now_ms;
                        }
                        if (now_ms - tracker->progress_ms > (uint32_t)MOTOR_NO_PROGRESS_S * 1000 ||
                            now_ms - tracker->move_start_ms > (uint32_t)MOTOR_MAX_RUN_S * 1000) {
                            hal_motor_stop();
                            tracker->motor_fault = true;
                            tracker->slot_move_active = false;
                            ESP_LOGE(TAG, "MOTOR FAULT: no progress toward %.2f deg (now %.2f) - stopped. "
                                          "Check direction (MOTOR_REVERSE) / motor / sensor.",
                                     (double)tracker->slot_target, (double)cur);
                        } else {
                            hal_motor_move_to_angle(tracker->slot_target, tracker->last_inc);
                        }
                    }
                } else {
                    hal_motor_stop();                        // between slots the panel is held still
                }
            }
            break;
        case TRACKER_STATE_NIGHT_RESET:
            if (fresh_inc) {
                if (tracker->last_inc_ok) {
                    hal_motor_move_to_angle(SOLAR_PARK_ANGLE, tracker->last_inc);
                } else {
                    hal_motor_stop();
                }
            }
            break;
        case TRACKER_STATE_MANUAL:
            if (hal_is_up_button_pressed() || tracker->web_up) {
                ESP_LOGI(TAG, "Manual UP (Web:%d Phys:%d)", tracker->web_up, hal_is_up_button_pressed());
                if (fresh_inc && tracker->last_inc_ok) {
                    if (!tracker->manual_target_seeded) {
                        tracker->manual_target = clamp_to_table(tracker, discrete_target);
                        tracker->manual_target_seeded = true;
                    }
                    tracker->manual_target += 0.1f;
                    if (tracker->manual_target > SOLAR_MAX_TILT) tracker->manual_target = SOLAR_MAX_TILT;
                    if (tracker->manual_target < -SOLAR_MAX_TILT) tracker->manual_target = -SOLAR_MAX_TILT;
                }
                hal_motor_move_up();
                tracker->web_override_active_angle = false;
            } else if (hal_is_down_button_pressed() || tracker->web_down) {
                ESP_LOGI(TAG, "Manual DOWN (Web:%d Phys:%d)", tracker->web_down, hal_is_down_button_pressed());
                if (fresh_inc && tracker->last_inc_ok) {
                    if (!tracker->manual_target_seeded) {
                        tracker->manual_target = clamp_to_table(tracker, discrete_target);
                        tracker->manual_target_seeded = true;
                    }
                    tracker->manual_target -= 0.1f;
                    if (tracker->manual_target > SOLAR_MAX_TILT) tracker->manual_target = SOLAR_MAX_TILT;
                    if (tracker->manual_target < -SOLAR_MAX_TILT) tracker->manual_target = -SOLAR_MAX_TILT;
                }
                hal_motor_move_down();
                tracker->web_override_active_angle = false;
            } else if (tracker->web_override_active_angle) {
                if (fresh_inc) {
                    if (tracker->last_inc_ok) {
                        hal_motor_move_to_angle(tracker->manual_target, tracker->last_inc);
                    } else {
                        hal_motor_stop();
                    }
                }
            } else {
                hal_motor_stop();
            }
            break;
        default:
            hal_motor_stop();
            break;
    }

    if (fresh_inc) log_angle_updates(tracker);

    tracker->prev_state = tracker->state;
    tracker->loop_count++;
}


/* Called every minute from the RTC task with the RTC time (minute aligned).
 * When that time reaches a new SolTrack table slot, start moving to the slot's angle. */
static void tracker_on_minute_impl(tracker_handle_t tracker, time_t minute_epoch) {
    tracker->last_tick_ms = hal_get_millis();
    if (!hal_time_is_trusted() || tracker->state != TRACKER_STATE_TRACKING) return;

    solar_config_t cfg;
    solar_math_get_config(&cfg);
    time_t day_start = local_day_start(minute_epoch, &cfg);
    ensure_table_cached(tracker, day_start);

    int idx = (int)((minute_epoch - day_start) / TRACKER_INTERVAL_SECONDS);
    if (idx < 0) idx = 0;
    if (idx >= TRACKER_TABLE_SIZE) idx = TRACKER_TABLE_SIZE - 1;

    if (idx != tracker->last_slot_idx) {
        start_slot_move(tracker, idx, tracker->cached_table_values[idx], "RTC time reached table slot");
    }
}

static void tracker_get_snapshot_impl(tracker_handle_t tracker, float *current_tilt, float *target_tilt,
                                      tracker_state_t *state, tracker_mode_t *effective_mode,
                                      bool *motor_fault, bool *inclinometer_ok) {
    if (current_tilt)    *current_tilt    = get_active_angle(tracker->last_inc);
    if (target_tilt)     *target_tilt     = tracker->state == TRACKER_STATE_STOW ? SOLAR_STOW_ANGLE :
                                            tracker->slot_move_active ? tracker->slot_target
                                                                       : get_active_angle(tracker->last_inc);
    if (state)            *state           = tracker->state;
    if (effective_mode)   *effective_mode  = tracker->effective_mode;
    if (motor_fault)      *motor_fault     = tracker->motor_fault;
    if (inclinometer_ok)  *inclinometer_ok = tracker->last_inc_ok;
}

static void tracker_set_web_buttons_impl(tracker_handle_t tracker, bool up, bool down) {
    tracker->web_up = up;
    tracker->web_down = down;
    if (up || down) {
        tracker->web_override_active_angle = false;
        tracker->web_mode = TRACKER_MODE_MANUAL;
        tracker->web_override = true;
    }
}

static cJSON* tracker_get_status_json_impl(tracker_handle_t tracker) {
    time_t t = hal_get_time();
    solar_config_t cfg;
    solar_math_get_config(&cfg);
    
    sun_position_t sun = solar_math_calculate_sun_position(t);
    
    // Calculate discrete target from table
    struct tm dt_local;
    time_t local_t = t + (time_t)(cfg.timezone_offset * 3600);
    gmtime_r(&local_t, &dt_local);
    
    // Day start in local time for table indexing
    struct tm midnight = dt_local;
    midnight.tm_hour = 0; midnight.tm_min = 0; midnight.tm_sec = 0;
    // timegm treats midnight as UTC. Subtract offset to get true UTC epoch of local midnight.
    time_t day_start_local_epoch = timegm(&midnight) - (time_t)(cfg.timezone_offset * 3600);
    
    ensure_table_cached(tracker, day_start_local_epoch);
    
    // day_start_local_epoch is the UTC epoch of the local midnight.
    float time_diff = (float)(t - day_start_local_epoch);
    float idx_f = time_diff / (float)TRACKER_INTERVAL_SECONDS;
    int idx = (int)idx_f;
    
    float discrete_target = 0;
    if (idx < 0) {
        discrete_target = tracker->cached_table_values[0];
    } else if (idx >= TRACKER_TABLE_SIZE) {
        discrete_target = tracker->cached_table_values[TRACKER_TABLE_SIZE - 1];
    } else {
        discrete_target = tracker->cached_table_values[idx];
    }

    float effective_target;
    if (tracker->state == TRACKER_STATE_STOW) {
        effective_target = SOLAR_STOW_ANGLE;
    } else if (tracker->effective_mode == TRACKER_MODE_MANUAL) {
        effective_target = tracker->manual_target;
    } else {
        effective_target = clamp_to_table(tracker, discrete_target);
    }
    
    // Use cached inclinometer data
    inclinometer_data_t inc = tracker->last_inc;
    float current_tilt = get_active_angle(inc);

    char diag[256];
    hal_get_diagnostics(diag, sizeof(diag));
    char diag_with_inc[256];
    const char *inc_suffix_fmt = " | Inc[x:%.2f y:%.2f z:%.2f]";
    int suffix_len = snprintf(NULL, 0, inc_suffix_fmt, inc.x, inc.y, inc.z);
    int max_diag = 0;
    if (suffix_len < (int)sizeof(diag_with_inc) - 1) {
        max_diag = (int)sizeof(diag_with_inc) - 1 - suffix_len;
    }
    snprintf(diag_with_inc, sizeof(diag_with_inc), "%.*s", max_diag, diag);
    if (suffix_len < (int)sizeof(diag_with_inc) - 1) {
        size_t used = strlen(diag_with_inc);
        snprintf(diag_with_inc + used, sizeof(diag_with_inc) - used, inc_suffix_fmt, inc.x, inc.y, inc.z);
    }

    double alpha_rad = sun.elevation * M_PI / 180.0;
    double phi_rad = sun.azimuth * M_PI / 180.0;
    double sx = cos(alpha_rad) * sin(phi_rad);
    double sz = sin(alpha_rad);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "state", state_to_str(tracker->state));
    cJSON_AddStringToObject(root, "mode", tracker->effective_mode == TRACKER_MODE_AUTO ? "AUTO" : "MANUAL");
    cJSON_AddNumberToObject(root, "time", (double)t);
    cJSON_AddNumberToObject(root, "tz_offset", cfg.timezone_offset);
    cJSON_AddStringToObject(root, "tz_name", cfg.timezone_name);
    cJSON_AddNumberToObject(root, "sun_elevation", sun.elevation);
    cJSON_AddNumberToObject(root, "sun_azimuth", sun.azimuth);
    cJSON_AddNumberToObject(root, "sx", sx);
    cJSON_AddNumberToObject(root, "sz", sz);
    cJSON_AddNumberToObject(root, "current_tilt", current_tilt);
    cJSON_AddNumberToObject(root, "target_tilt", effective_target);
    cJSON_AddNumberToObject(root, "sensor_x", inc.x);
    cJSON_AddNumberToObject(root, "sensor_y", inc.y);
    cJSON_AddNumberToObject(root, "sensor_z", inc.z);
    cJSON_AddNumberToObject(root, "lat", cfg.latitude);
    cJSON_AddNumberToObject(root, "lon", cfg.longitude);
    cJSON_AddStringToObject(root, "diagnostics", diag_with_inc);
    cJSON_AddNumberToObject(root, "clean_angle", SOLAR_CLEAN_ANGLE);
    cJSON_AddNumberToObject(root, "maint_angle", MAINTENANCE_ANGLE);
    cJSON_AddBoolToObject(root, "wind_stow", tracker->wind_stow);
    cJSON_AddNumberToObject(root, "max_tilt", SOLAR_MAX_TILT);
    cJSON_AddNumberToObject(root, "panel_width", solar_math_get_panel_width());
    cJSON_AddNumberToObject(root, "pitch_distance", solar_math_get_pitch_distance());
    cJSON_AddBoolToObject(root, "time_synced", t >= 1700000000);
    
    // Compute per-entry correction status for the current sun position
    {
        tracking_tilt_result_t cur = solar_math_calculate_tracking_tilt_advanced(sun.elevation, sun.azimuth);
        cJSON_AddStringToObject(root, "tracking_mode", solar_math_tracking_mode_to_str(cur.mode));
        cJSON_AddBoolToObject(root, "correction_active", cur.correction_active);
    }
    
    cJSON_AddItemToObject(root, "table", cJSON_Duplicate(tracker->cached_table_obj, true));

    // Next Move Prediction
    int next_idx = (idx + 1 > (TRACKER_TABLE_SIZE - 1)) ? (TRACKER_TABLE_SIZE - 1) : idx + 1;
    float target30 = tracker->cached_table_values[next_idx];
    target30 = clamp_to_table(tracker, target30);
    cJSON_AddNumberToObject(root, "target_30", target30);
    cJSON_AddNumberToObject(root, "target_30_time", (double)(day_start_local_epoch + (next_idx * TRACKER_INTERVAL_SECONDS)));

    rise_set_data_t rs = solar_math_calculate_rise_set(t);
    // Convert fractional hour to UTC timestamp
    cJSON_AddNumberToObject(root, "sunrise", (double)(day_start_local_epoch + (rs.sunrise * 3600)));
    cJSON_AddNumberToObject(root, "sunset", (double)(day_start_local_epoch + (rs.sunset * 3600)));
    
    return root;
}

static tracker_state_t tracker_get_state_impl(tracker_handle_t tracker) {
    return tracker->state;
}

static void tracker_set_mode_impl(tracker_handle_t tracker, tracker_mode_t mode) {
    tracker->web_mode = mode;
    tracker->web_override = true;
    if (mode == TRACKER_MODE_MANUAL) {
        wind_stow_cancel(tracker, "manual mode selected");
        if (tracker->last_inc_ok) {
            time_t t = hal_get_time();
            solar_config_t cfg;
            solar_math_get_config(&cfg);
            struct tm dt_local;
            time_t local_t = t + (time_t)(cfg.timezone_offset * 3600);
            gmtime_r(&local_t, &dt_local);
            struct tm midnight = dt_local;
            midnight.tm_hour = 0; midnight.tm_min = 0; midnight.tm_sec = 0;
            time_t day_start_local_epoch = timegm(&midnight) - (time_t)(cfg.timezone_offset * 3600);
            ensure_table_cached(tracker, day_start_local_epoch);

            float time_diff = (float)(t - day_start_local_epoch);
            float idx_f = time_diff / (float)TRACKER_INTERVAL_SECONDS;
            int idx = (int)idx_f;
            float discrete_target = 0;
            if (idx < 0) {
                discrete_target = tracker->cached_table_values[0];
            } else if (idx >= TRACKER_TABLE_SIZE) {
                discrete_target = tracker->cached_table_values[TRACKER_TABLE_SIZE - 1];
            } else {
                discrete_target = tracker->cached_table_values[idx];
            }
            tracker->manual_target = clamp_to_table(tracker, discrete_target);
            tracker->web_override_active_angle = true;
            tracker->manual_target_seeded = true;
        }
    }
}

static void tracker_set_manual_target_impl(tracker_handle_t tracker, float angle) {
    if (tracker->wind_stow) {
        ESP_LOGW(TAG, "Angle command %.2f deg ignored: wind stow hold active", (double)angle);
        return;
    }
    time_t t = hal_get_time();
    solar_config_t cfg;
    solar_math_get_config(&cfg);

    struct tm dt_local;
    time_t local_t = t + (time_t)(cfg.timezone_offset * 3600);
    gmtime_r(&local_t, &dt_local);
    struct tm midnight = dt_local;
    midnight.tm_hour = 0; midnight.tm_min = 0; midnight.tm_sec = 0;
    time_t day_start_local_epoch = timegm(&midnight) - (time_t)(cfg.timezone_offset * 3600);
    ensure_table_cached(tracker, day_start_local_epoch);

    tracker->manual_target = angle;
    if (tracker->manual_target > SOLAR_MAX_TILT) tracker->manual_target = SOLAR_MAX_TILT;
    if (tracker->manual_target < -SOLAR_MAX_TILT) tracker->manual_target = -SOLAR_MAX_TILT;
    tracker->manual_target_seeded = true;
    tracker->web_override_active_angle = true;
    tracker->web_override = true;
    tracker->web_mode = TRACKER_MODE_MANUAL;
    ESP_LOGI(TAG, "Angle command: set target %.2f deg (requested %.2f)",
             (double)tracker->manual_target, (double)angle);
}


static bool tracker_set_geometry_impl(tracker_handle_t tracker, float panel_width, float pitch_distance) {
    if (!solar_math_set_geometry(panel_width, pitch_distance)) return false;
    tracker->last_table_day = 0;      /* forces ensure_table_cached() to rebuild the day table */
    tracker->last_slot_idx = -1;      /* next RTC minute re-applies the current slot with the new geometry */
    return true;
}

/* ---- thread-safe public API (wrappers) ---- */
void tracker_get_snapshot(tracker_handle_t t, float *current_tilt, float *target_tilt,
                          tracker_state_t *state, tracker_mode_t *effective_mode,
                          bool *motor_fault, bool *inclinometer_ok) {
    TRK_LOCK();
    tracker_get_snapshot_impl(t, current_tilt, target_tilt, state, effective_mode, motor_fault, inclinometer_ok);
    TRK_UNLOCK();
}
void tracker_on_minute(tracker_handle_t t, time_t minute_epoch) { TRK_LOCK(); tracker_on_minute_impl(t, minute_epoch); TRK_UNLOCK(); }
void tracker_update(tracker_handle_t t) { TRK_LOCK(); tracker_update_impl(t); TRK_UNLOCK(); }
void tracker_set_web_buttons(tracker_handle_t t, bool up, bool down) { TRK_LOCK(); tracker_set_web_buttons_impl(t, up, down); TRK_UNLOCK(); }
cJSON* tracker_get_status_json(tracker_handle_t t) { TRK_LOCK(); cJSON *r = tracker_get_status_json_impl(t); TRK_UNLOCK(); return r; }
tracker_state_t tracker_get_state(tracker_handle_t t) { TRK_LOCK(); tracker_state_t r = tracker_get_state_impl(t); TRK_UNLOCK(); return r; }
void tracker_set_mode(tracker_handle_t t, tracker_mode_t m) { TRK_LOCK(); tracker_set_mode_impl(t, m); TRK_UNLOCK(); }
void tracker_set_manual_target(tracker_handle_t t, float a) { TRK_LOCK(); tracker_set_manual_target_impl(t, a); TRK_UNLOCK(); }
bool tracker_set_geometry(tracker_handle_t t, float w, float p) { TRK_LOCK(); bool r = tracker_set_geometry_impl(t, w, p); TRK_UNLOCK(); return r; }
