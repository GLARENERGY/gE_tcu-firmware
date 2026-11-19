#include "tcu_board_hw_interface.h"
#include "hal_tcu.h"
#include "AppConfig.h"
#include "inclinometer.h"
#include "pcf2131.h"
#include "rtc_service.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

static const char *TAG = "HAL";

#define TILT_MAX_AGE_MS   1000     /* older than this = sensor considered dead */

static i2c_master_bus_handle_t s_bus;
static QueueHandle_t           s_tilt_q;
static pcf2131_t               s_rtc;
static bool                    s_rtc_ready;
static volatile bool           s_time_trusted;

void hal_tcu_attach(i2c_master_bus_handle_t i2c_bus, QueueHandle_t tilt_queue)
{
    s_bus = i2c_bus;
    s_tilt_q = tilt_queue;
}

/* ---------------- System ---------------- */
void hal_debug_init(void) { /* logs stay on the default console (USB/UART0) on the C6 */ }
uint32_t hal_get_millis(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
void hal_delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

/* ---------------- Time (PCF2131) ---------------- */
void hal_rtc_init(void)
{
    setenv("TZ", "UTC", 1);
    tzset();

    if (!s_bus || pcf2131_init(&s_rtc, s_bus, 100000) != ESP_OK) {
        ESP_LOGE(TAG, "PCF2131 init failed (no I2C bus or device)");
        return;
    }
    if (pcf2131_is_present(s_bus) != ESP_OK) {
        ESP_LOGE(TAG, "PCF2131 not found at 0x%02X", PCF2131_I2C_ADDR);
        return;
    }
    s_rtc_ready = true;          /* time validity + system time are handled in rtc_service_start() */
}

time_t hal_get_time(void) { return time(NULL); }

bool hal_rtc_is_valid(void)
{
    if (!s_rtc_ready) return false;
    pcf2131_time_t t; bool osf = true;
    if (pcf2131_get_time(&s_rtc, &t, &osf) != ESP_OK) return false;
    return !osf && t.year > 2024;
}

void hal_set_time(time_t t)
{
    if (s_rtc_ready) {
        pcf2131_time_t rt;
        pcf2131_from_epoch(t, &rt);
        if (pcf2131_set_time(&s_rtc, &rt) != ESP_OK) ESP_LOGE(TAG, "FAILED to set RTC time");
        else ESP_LOGI(TAG, "RTC updated");
    }
    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&tv, NULL);
    s_time_trusted = true;           /* set from browser / SNTP: real time */
    rtc_service_on_time_set();       /* re-arm alarm for the next full minute */
}

pcf2131_t *hal_tcu_get_rtc(void) { return s_rtc_ready ? &s_rtc : NULL; }
void hal_tcu_set_time_trusted(bool trusted) { s_time_trusted = trusted; }
bool hal_time_is_trusted(void) { return s_time_trusted; }

/* ---------------- Inclinometer (LIS2DW12 via queue) ---------------- */
void hal_inclinometer_init(void) { /* sensor task is started from app_main */ }

bool hal_inclinometer_read(inclinometer_data_t *data)
{
    tilt_msg_t m;
    if (!s_tilt_q || xQueuePeek(s_tilt_q, &m, 0) != pdTRUE) return false;
    if (!tilt_msg_is_fresh(&m, TILT_MAX_AGE_MS)) return false;   /* invalid or stale -> motor stops */

    data->x = m.x; data->y = m.y; data->z = m.z;

    float *active = NULL;
#if INCLINOMETER_AXIS == 0
    active = &data->x;
#elif INCLINOMETER_AXIS == 1
    active = &data->y;
#elif INCLINOMETER_AXIS == 2
    active = &data->z;
#endif
    if (active) {
#ifdef INCLINOMETER_REVERSE            /* defineparse emits a bare "#define" for true, nothing for false */
        *active = -(*active);
#endif
        *active += INCLINOMETER_OFFSET;
    }
    return true;
}

/* ---------------- Motors ---------------- */
void hal_motor_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_MOTOR_PLUS) | (1ULL << PIN_MOTOR_MINUS),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    hal_motor_stop();
}

/* Motor truth table: A=0,B=1 one direction | A=1,B=0 the other | A=0,B=0 stopped.
 * A = PIN_MOTOR_PLUS, B = PIN_MOTOR_MINUS. "Up" = A=1,B=0 (set MOTOR_REVERSE in config.yaml to swap).
 * The line that turns off is cleared first (break before make). */
#ifdef MOTOR_REVERSE
#define PIN_UP_ON    PIN_MOTOR_MINUS
#define PIN_UP_OFF   PIN_MOTOR_PLUS
#else
#define PIN_UP_ON    PIN_MOTOR_PLUS
#define PIN_UP_OFF   PIN_MOTOR_MINUS
#endif

void hal_motor_move_up(void)   { gpio_set_level(PIN_UP_OFF, 0); gpio_set_level(PIN_UP_ON, 1); }
void hal_motor_move_down(void) { gpio_set_level(PIN_UP_ON, 0);  gpio_set_level(PIN_UP_OFF, 1); }
void hal_motor_stop(void)      { gpio_set_level(PIN_MOTOR_PLUS, 0); gpio_set_level(PIN_MOTOR_MINUS, 0); }

void hal_motor_move_to_angle(float target_angle, inclinometer_data_t inc)
{
    float current = 0;
#if INCLINOMETER_AXIS == 0
    current = inc.x;
#elif INCLINOMETER_AXIS == 1
    current = inc.y;
#elif INCLINOMETER_AXIS == 2
    current = inc.z;
#endif
    float diff = target_angle - current;
    if (fabsf(diff) < MOTOR_DEADBAND) hal_motor_stop();
    else if (diff > 0)                hal_motor_move_up();
    else                              hal_motor_move_down();
}

bool hal_is_motor_moving_up(void)   { return gpio_get_level(PIN_UP_ON) == 1 && gpio_get_level(PIN_UP_OFF) == 0; }
bool hal_is_motor_moving_down(void) { return gpio_get_level(PIN_UP_ON) == 0 && gpio_get_level(PIN_UP_OFF) == 1; }

/* ---------------- UI ---------------- */
void hal_ui_init(void)
{
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_MODE_TOGGLE) | (1ULL << PIN_UP_BUTTON) | (1ULL << PIN_DOWN_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,       /* switches/buttons are active-low */
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);
}

/* PIN_MODE_TOGGLE is a momentary push-button (not a maintained switch): each press
 * toggles between AUTO and MANUAL. This is called once per tracker_update() cycle
 * (~10 Hz), which is slow enough that a single read per call is already debounced -
 * the edge check below just makes sure a held button produces exactly one toggle. */
tracker_mode_t hal_get_mode_switch(void)
{
    static tracker_mode_t s_mode = TRACKER_MODE_AUTO;   /* default at boot */
    static bool s_was_pressed = false;

    bool pressed = (gpio_get_level(PIN_MODE_TOGGLE) == 0);   /* active low */
    if (pressed && !s_was_pressed) {
        s_mode = (s_mode == TRACKER_MODE_AUTO) ? TRACKER_MODE_MANUAL : TRACKER_MODE_AUTO;
        ESP_LOGI(TAG, "Mode button pressed -> %s", s_mode == TRACKER_MODE_AUTO ? "AUTO" : "MANUAL");
    }
    s_was_pressed = pressed;
    return s_mode;
}
bool hal_is_up_button_pressed(void)   { return gpio_get_level(PIN_UP_BUTTON) == 0; }
bool hal_is_down_button_pressed(void) { return gpio_get_level(PIN_DOWN_BUTTON) == 0; }
void hal_set_mode_indicator(tracker_mode_t mode) { (void)mode; }

void hal_get_diagnostics(char *buf, size_t max_len)
{
    snprintf(buf, max_len, "RTC:%s Up:%lum Heap:%luKB Pins[M:%d U:%d D:%d]",
             hal_rtc_is_valid() ? "OK" : "FAIL",
             (unsigned long)(hal_get_millis() / 60000),
             (unsigned long)(esp_get_free_heap_size() / 1024),
             gpio_get_level(PIN_MODE_TOGGLE),
             gpio_get_level(PIN_UP_BUTTON),
             gpio_get_level(PIN_DOWN_BUTTON));
}
