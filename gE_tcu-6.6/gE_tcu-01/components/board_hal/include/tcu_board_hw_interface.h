#ifndef HAL_H
#define HAL_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

typedef enum {
    TRACKER_MODE_AUTO,
    TRACKER_MODE_MANUAL,
    TRACKER_MODE_STOW
} tracker_mode_t;

// --- Time Management ---
void hal_rtc_init(void);
bool hal_rtc_is_valid(void);
bool hal_time_is_trusted(void);   // true once the time is real (valid RTC, or set from browser/SNTP)
time_t hal_get_time(void);
void hal_set_time(time_t t);

// --- Inclinometer (RS485/Modbus) ---
typedef struct {
    float x;
    float y;
    float z;
} inclinometer_data_t;

void hal_inclinometer_init(void);
bool hal_inclinometer_read(inclinometer_data_t *data);

// --- Actuators (Motors) ---
void hal_motor_init(void);
void hal_motor_move_up(void);
void hal_motor_move_down(void);
void hal_motor_stop(void);
void hal_motor_move_to_angle(float target_angle, inclinometer_data_t current_inc);
bool hal_is_motor_moving_up(void);
bool hal_is_motor_moving_down(void);

// --- UI / Sensors ---
void hal_ui_init(void);
tracker_mode_t hal_get_mode_switch(void);
bool hal_is_up_button_pressed(void);
bool hal_is_down_button_pressed(void);
void hal_set_mode_indicator(tracker_mode_t mode);
void hal_get_diagnostics(char *buf, size_t max_len);

// --- System ---
void hal_debug_init(void);
uint32_t hal_get_millis(void);
void hal_delay_ms(uint32_t ms);

#endif // HAL_H
