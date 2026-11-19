#include <math.h>
#include "esp_log.h"
#include "inclinometer.h"

static const char *TAG = "incl";

/* ======================= LIS2DW12 low-level driver ======================= */
#define REG_WHO_AM_I   0x0F
#define REG_CTRL1      0x20
#define REG_CTRL2      0x21
#define REG_CTRL6      0x25
#define REG_STATUS     0x27           /* OUT_X_L..OUT_Z_H follow at 0x28..0x2D */

#define WHO_AM_I_VAL      0x44
#define CTRL2_SOFT_RESET  0x40
#define CTRL2_BDU         0x08
#define CTRL2_IF_ADD_INC  0x04
#define STATUS_DRDY       0x01

/* CTRL1: ODR=0100 (50 Hz) | MODE=01 (high performance) | LP_MODE=00 */
#define CTRL1_50HZ_HP      ((0x4 << 4) | (0x1 << 2))
/* CTRL6: BW_FILT=00 | FS=00 (+/-2 g) | FDS=0 | LOW_NOISE=1 */
#define CTRL6_2G_LOWNOISE  0x04
#define SENS_G_PER_LSB_2G  0.000244f  /* 0.244 mg/LSB, 14-bit high-performance */
#define I2C_TO_MS          100

static i2c_master_bus_handle_t  s_bus;
static i2c_master_dev_handle_t  s_dev;
static QueueHandle_t            s_queue;
static TaskHandle_t             s_task;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_dev, b, 2, I2C_TO_MS);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, len, I2C_TO_MS);
}

static esp_err_t sensor_init(void)
{
    if (s_dev == NULL) {                     /* add the device only once */
        i2c_device_config_t cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address  = ACCEL_I2C_ADDR,
            .scl_speed_hz    = ACCEL_I2C_HZ,
        };
        esp_err_t e = i2c_master_bus_add_device(s_bus, &cfg, &s_dev);
        if (e != ESP_OK) return e;
    }

    uint8_t id = 0;
    esp_err_t err = reg_read(REG_WHO_AM_I, &id, 1);
    if (err != ESP_OK) return err;
    if (id != WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I=0x%02X (expected 0x%02X)", id, WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }

    if ((err = reg_write(REG_CTRL2, CTRL2_SOFT_RESET)) != ESP_OK) return err;
    for (int i = 0; i < 10; i++) {           /* wait for the reset bit to clear */
        vTaskDelay(pdMS_TO_TICKS(2));
        uint8_t c2 = CTRL2_SOFT_RESET;
        if (reg_read(REG_CTRL2, &c2, 1) == ESP_OK && !(c2 & CTRL2_SOFT_RESET)) break;
    }

    if ((err = reg_write(REG_CTRL2, CTRL2_BDU | CTRL2_IF_ADD_INC)) != ESP_OK) return err;
    if ((err = reg_write(REG_CTRL6, CTRL6_2G_LOWNOISE)) != ESP_OK) return err;
    if ((err = reg_write(REG_CTRL1, CTRL1_50HZ_HP)) != ESP_OK) return err;   /* starts sampling */

    ESP_LOGI(TAG, "LIS2DW12 ready @0x%02X (+/-2g, 50Hz, HP)", ACCEL_I2C_ADDR);
    return ESP_OK;
}

/* One burst: STATUS + OUT_X/Y/Z. g[] in g. *fresh = DRDY bit. */
static esp_err_t sensor_read_g(float g[3], bool *fresh)
{
    uint8_t b[7];
    esp_err_t err = reg_read(REG_STATUS, b, sizeof(b));
    if (err != ESP_OK) return err;

    *fresh = (b[0] & STATUS_DRDY) != 0;
    for (int i = 0; i < 3; i++) {
        int16_t raw = (int16_t)((uint16_t)b[1 + 2 * i] | ((uint16_t)b[2 + 2 * i] << 8));
        raw >>= 2;                           /* 14-bit data is left-justified */
        g[i] = raw * SENS_G_PER_LSB_2G;
    }
    return ESP_OK;
}

/* Gravity vector -> per-axis tilt in degrees */
static void g_to_tilt(const float g[3], float *x, float *y, float *z)
{
    const float r2d = 180.0f / (float)M_PI;
    *x = atan2f(g[0], sqrtf(g[1] * g[1] + g[2] * g[2])) * r2d;
    *y = atan2f(g[1], sqrtf(g[0] * g[0] + g[2] * g[2])) * r2d;
    *z = atan2f(g[2], sqrtf(g[0] * g[0] + g[1] * g[1])) * r2d;
}

/* ============================== Task ============================== */
#define SAMPLE_PERIOD_MS    20
#define FAULT_AFTER_FAILS   5            /* consecutive failures -> publish invalid */
#define REINIT_AFTER_FAILS  25
#define G_MIN               0.70f        /* reject samples when |a| is far from 1 g */
#define G_MAX               1.30f

static void publish(float x, float y, float z, bool valid)
{
    tilt_msg_t m = { .x = x, .y = y, .z = z, .valid = valid, .tick = xTaskGetTickCount() };
    xQueueOverwrite(s_queue, &m);
}

static void inclinometer_task(void *arg)
{
    float sum[3] = {0};
    int   n = 0, fails = 0;
    bool  ready = false;

    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        if (ulTaskNotifyTake(pdTRUE, 0)) { sum[0] = sum[1] = sum[2] = 0; n = 0; }   /* triggered: start a fresh window */

        if (!ready) {
            if (sensor_init() == ESP_OK) {
                ready = true; fails = 0; n = 0; sum[0] = sum[1] = sum[2] = 0;
            } else {
                publish(0, 0, 0, false);
                ESP_LOGW(TAG, "LIS2DW12 init failed, retrying");
                vTaskDelay(pdMS_TO_TICKS(2000));
                last = xTaskGetTickCount();
            }
            continue;
        }

        float g[3]; bool fresh = false;
        if (sensor_read_g(g, &fresh) != ESP_OK) {
            fails++;
            if (fails == FAULT_AFTER_FAILS) { publish(0, 0, 0, false); ESP_LOGW(TAG, "sensor read failing"); }
            if (fails >= REINIT_AFTER_FAILS) ready = false;
            continue;
        }
        fails = 0;
        if (!fresh) continue;

        float mag = sqrtf(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
        if (mag < G_MIN || mag > G_MAX) {        /* shock / vibration / bad data */
            ESP_LOGD(TAG, "rejected sample |a|=%.2f g", mag);
            continue;
        }

        sum[0] += g[0]; sum[1] += g[1]; sum[2] += g[2];
        if (++n >= ACCEL_AVG_SAMPLES) {
            float avg[3] = { sum[0] / n, sum[1] / n, sum[2] / n };
            float x, y, z;
            g_to_tilt(avg, &x, &y, &z);
            publish(x, y, z, true);
            ESP_LOGD(TAG, "tilt x=%.2f y=%.2f z=%.2f", x, y, z);
            sum[0] = sum[1] = sum[2] = 0; n = 0;
        }
    }
}

esp_err_t inclinometer_start(i2c_master_bus_handle_t bus, QueueHandle_t queue)
{
    if (!bus || !queue) return ESP_ERR_INVALID_ARG;
    s_bus = bus; s_queue = queue;
    return xTaskCreate(inclinometer_task, "incl", 4096, NULL, 5, &s_task) == pdPASS ? ESP_OK : ESP_FAIL;
}

void inclinometer_trigger(void)
{
    if (s_task) xTaskNotifyGive(s_task);
}
