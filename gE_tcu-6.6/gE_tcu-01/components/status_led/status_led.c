/* Built-in RGB LED (single WS2812) status indicator - see status_led.h for the colour table.
 * Self-contained RMT driver (no external component needed). */
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"

#include "AppConfig.h"
#include "tcu_board_hw_interface.h"
#include "tracker_core.h"
#include "status_led.h"

#ifndef RGB_LED_PIN
#define RGB_LED_PIN 8               /* ESP32-C6-DevKitM-1 on-board WS2812 */
#endif

static const char *TAG = "LED";

/* ---- Colours (full-intensity RGB, scaled by the brightness fraction below) ---- */
typedef struct { uint8_t r, g, b; } rgb_t;
static const rgb_t COL_LIGHT_GREEN = { 110, 255,  70 };
static const rgb_t COL_PURPLE      = { 150,   0, 255 };
static const rgb_t COL_RED         = { 255,   0,   0 };
static const rgb_t COL_YELLOW      = { 255, 170,   0 };
static const rgb_t COL_BLUE        = {   0,   0, 255 };
static const rgb_t COL_OFF         = {   0,   0,   0 };

/* ---- Brightness fractions ---- */
#define BR_THIRD       (1.0f / 3.0f)
#define BR_TWO_THIRDS  (2.0f / 3.0f)
#define BR_HALF        0.5f

#define BLINK_HALF_PERIOD_MS  500       /* blue blink: 1 Hz */
#define STARTUP_GRACE_MS      5000      /* inclinometer needs a moment after boot before "not detected" is shown */
#define LED_LOOP_MS           50

static rmt_channel_handle_t s_chan;
static rmt_encoder_handle_t s_enc;

static tracker_handle_t            s_tracker;
static status_led_web_connected_fn s_web_connected;

static esp_err_t led_hw_init(void)
{
    rmt_tx_channel_config_t tx = {
        .gpio_num = RGB_LED_PIN,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,      /* 0.1 us per tick */
        .mem_block_symbols = 48,
        .trans_queue_depth = 2,
    };
    esp_err_t err = rmt_new_tx_channel(&tx, &s_chan);
    if (err != ESP_OK) return err;

    /* WS2812: bit0 = 0.3 us high / 0.9 us low, bit1 = 0.9 us high / 0.3 us low, MSB first (GRB order) */
    rmt_bytes_encoder_config_t be = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    err = rmt_new_bytes_encoder(&be, &s_enc);
    if (err != ESP_OK) return err;
    return rmt_enable(s_chan);
}

static void led_write(rgb_t c, float brightness)
{
    uint8_t grb[3] = {
        (uint8_t)((float)c.g * brightness + 0.5f),
        (uint8_t)((float)c.r * brightness + 0.5f),
        (uint8_t)((float)c.b * brightness + 0.5f),
    };
    rmt_transmit_config_t cfg = { .loop_count = 0 };
    if (rmt_transmit(s_chan, s_enc, grb, sizeof(grb), &cfg) == ESP_OK)
        rmt_tx_wait_all_done(s_chan, pdMS_TO_TICKS(20));   /* line then idles low = WS2812 latch */
}

static void led_task(void *arg)
{
    uint32_t start_ms = hal_get_millis();
    rgb_t   last_c = { 1, 2, 3 };            /* impossible first value: forces the first write */
    float   last_b = -1.0f;
    uint32_t last_write_ms = 0;

    for (;;) {
        float cur, tgt; tracker_state_t st; tracker_mode_t md; bool fault = false, inc_ok = true;
        tracker_get_snapshot(s_tracker, &cur, &tgt, &st, &md, &fault, &inc_ok);

        bool web      = s_web_connected ? s_web_connected() : false;
        bool moving   = hal_is_motor_moving_up() || hal_is_motor_moving_down();
        bool clock_ok = hal_time_is_trusted();
        uint32_t now  = hal_get_millis();
        bool grace    = (now - start_ms) < STARTUP_GRACE_MS;

        rgb_t col; float br;
        if (!inc_ok && !grace) {
            /* 3 / 4: inclinometer not detected */
            if (web) { col = COL_YELLOW; br = BR_HALF;  }
            else     { col = COL_RED;    br = BR_THIRD; }
        } else if (!clock_ok && !web) {
            /* 5: clock not set and nobody connected -> blink blue */
            bool on = ((now / BLINK_HALF_PERIOD_MS) & 1u) == 0;
            col = on ? COL_BLUE : COL_OFF;
            br  = BR_HALF;
        } else {
            /* 1 / 2: normal operation */
            col = web ? COL_PURPLE : COL_LIGHT_GREEN;
            br  = moving ? BR_TWO_THIRDS : BR_THIRD;
        }

        bool changed = (col.r != last_c.r || col.g != last_c.g || col.b != last_c.b || br != last_b);
        if (changed || (now - last_write_ms) > 1000) {      /* periodic refresh recovers from a glitched frame */
            led_write(col, br);
            last_c = col; last_b = br; last_write_ms = now;
        }
        vTaskDelay(pdMS_TO_TICKS(LED_LOOP_MS));
    }
}

esp_err_t status_led_start(tracker_handle_t tracker, status_led_web_connected_fn web_connected)
{
    s_tracker = tracker;
    s_web_connected = web_connected;
    esp_err_t err = led_hw_init();
    if (err != ESP_OK) { ESP_LOGE(TAG, "RGB LED init failed: %s", esp_err_to_name(err)); return err; }
    xTaskCreate(led_task, "status_led", 3072, NULL, 3, NULL);
    ESP_LOGI(TAG, "Status LED on GPIO %d", RGB_LED_PIN);
    return ESP_OK;
}
