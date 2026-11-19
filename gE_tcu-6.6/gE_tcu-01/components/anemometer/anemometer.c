#include "anemometer.h"
#include "AppConfig.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "WIND";

#ifndef ANEMOMETER_UART
#define ANEMOMETER_UART     UART_NUM_1        /* UART0 is the console on the ESP32-C6 */
#endif
#ifndef ANEMOMETER_BAUD
#define ANEMOMETER_BAUD     4800              /* sensor factory default */
#endif
#ifndef ANEMOMETER_ADDR
#define ANEMOMETER_ADDR     1                 /* sensor factory default */
#endif
#define POLL_PERIOD_MS      750               /* + up to 250 ms reply wait = about 1 s per poll */
#define REPLY_TIMEOUT_MS    250
#define STALE_MS            5000              /* no good reading for this long -> report "offline" */
#define FAILS_BEFORE_WARN   3
#define MAX_RAW             700               /* 70.0 m/s = top of the sensor range */

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static float        s_speed;
static TickType_t   s_tick;
static bool         s_have_reading;

/* Standard Modbus CRC-16 (poly 0xA001, init 0xFFFF), sent low byte first. */
static uint16_t crc16(const uint8_t *buf, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
    }
    return crc;
}

/* One query/reply. ESP_OK and *speed set on a valid, CRC-checked answer. */
static esp_err_t read_wind(float *speed)
{
    uint8_t req[8] = { ANEMOMETER_ADDR, 0x03, 0x00, 0x00, 0x00, 0x01, 0, 0 };
    uint16_t c = crc16(req, 6);
    req[6] = (uint8_t)(c & 0xFF);
    req[7] = (uint8_t)(c >> 8);

    uart_flush_input(ANEMOMETER_UART);
    uart_write_bytes(ANEMOMETER_UART, req, sizeof(req));
    uart_wait_tx_done(ANEMOMETER_UART, pdMS_TO_TICKS(50));

    /* 16 bytes of room: the 7-byte answer, plus an 8-byte echo of our own query on RS485 modules
     * that loop the transmitted bytes back. We search for the answer instead of assuming where it is. */
    uint8_t buf[16];
    int n = uart_read_bytes(ANEMOMETER_UART, buf, sizeof(buf), pdMS_TO_TICKS(REPLY_TIMEOUT_MS));
    for (int i = 0; i + 7 <= n; i++) {
        if (buf[i] != ANEMOMETER_ADDR || buf[i + 1] != 0x03 || buf[i + 2] != 0x02) continue;
        uint16_t got = (uint16_t)(buf[i + 5] | (buf[i + 6] << 8));
        if (crc16(&buf[i], 5) != got) continue;                       /* corrupted frame */
        uint16_t raw = (uint16_t)((buf[i + 3] << 8) | buf[i + 4]);
        if (raw > MAX_RAW) return ESP_ERR_INVALID_RESPONSE;
        *speed = raw / 10.0f;
        return ESP_OK;
    }
    return n > 0 ? ESP_ERR_INVALID_RESPONSE : ESP_ERR_TIMEOUT;
}

static void anemometer_task(void *arg)
{
    int fails = 0;
    for (;;) {
        float v;
        esp_err_t err = read_wind(&v);
        if (err == ESP_OK) {
            if (fails >= FAILS_BEFORE_WARN) ESP_LOGI(TAG, "anemometer back online");
            fails = 0;
            portENTER_CRITICAL(&s_mux);
            s_speed = v; s_tick = xTaskGetTickCount(); s_have_reading = true;
            portEXIT_CRITICAL(&s_mux);
        } else if (++fails == FAILS_BEFORE_WARN) {
            ESP_LOGW(TAG, "no valid reply (%s) - check A/B wiring, baud %d, address %d, 10-30 V supply",
                     esp_err_to_name(err), ANEMOMETER_BAUD, ANEMOMETER_ADDR);
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_PERIOD_MS));
    }
}

esp_err_t anemometer_start(void)
{
    const uart_config_t cfg = {
        .baud_rate  = ANEMOMETER_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_driver_install(ANEMOMETER_UART, 256, 0, 0, NULL, 0);
    if (err != ESP_OK) return err;
    if ((err = uart_param_config(ANEMOMETER_UART, &cfg)) != ESP_OK) return err;
    if ((err = uart_set_pin(ANEMOMETER_UART, ANEMOMETER_TX_PIN, ANEMOMETER_RX_PIN,
                            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)) != ESP_OK) return err;

    ESP_LOGI(TAG, "Anemometer on UART%d (TX=GPIO%d RX=GPIO%d, %d baud, address %d)",
             (int)ANEMOMETER_UART, ANEMOMETER_TX_PIN, ANEMOMETER_RX_PIN, ANEMOMETER_BAUD, ANEMOMETER_ADDR);
    return xTaskCreate(anemometer_task, "wind", 3072, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}

bool anemometer_get(float *speed_ms)
{
    bool ok;
    portENTER_CRITICAL(&s_mux);
    ok = s_have_reading && (xTaskGetTickCount() - s_tick) <= pdMS_TO_TICKS(STALE_MS);
    if (ok && speed_ms) *speed_ms = s_speed;
    portEXIT_CRITICAL(&s_mux);
    return ok;
}
