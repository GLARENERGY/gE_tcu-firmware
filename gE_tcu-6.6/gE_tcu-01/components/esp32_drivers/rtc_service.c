#include "rtc_service.h"
#include "hal_tcu.h"
#include "AppConfig.h"
#include "inclinometer.h"
#include "pcf2131.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <sys/time.h>

static const char *TAG = "RTC_SVC";

/* TESTING: 1 = always reset the RTC to 00:00:00 at boot (loses real time). 0 = only if invalid. */
#ifndef RTC_RESET_TO_ZERO_ON_BOOT
#define RTC_RESET_TO_ZERO_ON_BOOT 0
#endif
#define RTC_DEFAULT_YEAR   2026       /* date used when the RTC is reset to 00:00:00 */
#define RTC_DEFAULT_MONTH  1
#define RTC_DEFAULT_DAY    1
#define ALARM_WATCHDOG_MS  90000      /* no alarm for this long -> re-arm (recovers a lost alarm) */

static TaskHandle_t      s_task;
static pcf2131_t        *s_rtc;
static rtc_minute_cb_t   s_cb;
static volatile bool     s_resync_req;

/* ISR: keep it minimal - just wake the task. No I2C here. */
static void IRAM_ATTR inta_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    if (s_task) vTaskNotifyGiveFromISR(s_task, &hp);
    if (hp) portYIELD_FROM_ISR();
}

/* Arm the alarm for the next full minute after `now`; verify we did not miss it. */
static esp_err_t arm_next_minute(time_t now)
{
    for (int attempt = 0; attempt < 3; attempt++) {
        time_t next = (now / 60 + 1) * 60;
        pcf2131_time_t n;
        pcf2131_from_epoch(next, &n);

        esp_err_t err = pcf2131_set_alarm_hm(s_rtc, n.hour, n.minute);
        if (err == ESP_OK) err = pcf2131_alarm_clear_flag(s_rtc);   /* after the new alarm is loaded */
        if (err != ESP_OK) return err;

        pcf2131_time_t chk; bool osf;
        if (pcf2131_get_time(s_rtc, &chk, &osf) != ESP_OK) return ESP_OK;
        time_t now2 = pcf2131_to_epoch(&chk);
        if (now2 < next) {
            ESP_LOGD(TAG, "next alarm %02u:%02u:00", n.hour, n.minute);
            return ESP_OK;
        }
        now = now2;                                  /* minute rolled while writing -> retry */
    }
    return ESP_FAIL;
}

static void rtc_task(void *arg)
{
    for (;;) {
        uint32_t got = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ALARM_WATCHDOG_MS));

        pcf2131_time_t t; bool osf = false;
        if (pcf2131_get_time(s_rtc, &t, &osf) != ESP_OK) {
            ESP_LOGE(TAG, "RTC read failed");
            continue;
        }
        time_t now = pcf2131_to_epoch(&t);

        if (s_resync_req) {                          /* time was just (re)set: arm next full minute */
            s_resync_req = false;
            ESP_LOGI(TAG, "Time synchronised -> alarm armed for next full minute");
            arm_next_minute(now);
            continue;
        }
        if (!got) {                                  /* watchdog: the alarm never came */
            ESP_LOGW(TAG, "No RTC alarm for %d s -> re-arming", ALARM_WATCHDOG_MS / 1000);
            arm_next_minute(now);
            continue;
        }

        /* ---- alarm fired: this is the work the ISR hands over ---- */
        time_t minute_epoch = now - (now % 60);
        bool trusted = hal_time_is_trusted();

        if (arm_next_minute(now) != ESP_OK) ESP_LOGE(TAG, "failed to arm next alarm");
        if (gpio_get_level(RTC_INTA_PIN) == 0) {     /* pin still low -> AF re-set; avoid a missed edge */
            xTaskNotifyGive(s_task);
        }

        if (trusted) {                               /* RTC is the time master: keep system time aligned */
            struct timeval tv; gettimeofday(&tv, NULL);
            if (labs((long)(tv.tv_sec - now)) > 1) {
                struct timeval set = { .tv_sec = now, .tv_usec = 0 };
                settimeofday(&set, NULL);
            }
        }

        inclinometer_trigger();                      /* fresh angle measurement */
        ESP_LOGI(TAG, "tick %04u-%02u-%02u %02u:%02u:%02u UTC (trusted=%d)",
                 t.year, t.month, t.day, t.hour, t.minute, t.second, trusted);
        if (s_cb) s_cb(minute_epoch, trusted);
    }
}

esp_err_t rtc_service_start(rtc_minute_cb_t cb)
{
    s_rtc = hal_tcu_get_rtc();
    if (!s_rtc) return ESP_ERR_INVALID_STATE;
    s_cb = cb;

    esp_err_t err = pcf2131_config_24h(s_rtc);
    if (err != ESP_OK) { ESP_LOGE(TAG, "RTC not responding"); return err; }

    /* ---- RTC initialisation ---- */
    pcf2131_time_t t; bool osf = true;
    esp_err_t terr = pcf2131_get_time(s_rtc, &t, &osf);
    bool valid = (terr == ESP_OK) && !osf && t.year >= 2025;

    if (RTC_RESET_TO_ZERO_ON_BOOT || !valid) {
        pcf2131_time_t z = { .year = RTC_DEFAULT_YEAR, .month = RTC_DEFAULT_MONTH,
                             .day = RTC_DEFAULT_DAY, .hour = 0, .minute = 0, .second = 0 };
        if ((err = pcf2131_set_time(s_rtc, &z)) != ESP_OK) return err;
        hal_tcu_set_time_trusted(false);             /* placeholder time: tracker must not use it */
        ESP_LOGW(TAG, "RTC reset to 00:00:00 (not real time) - waiting for browser/SNTP time");
        t = z;
    } else {
        pcf2131_load_system_time(s_rtc);
        hal_tcu_set_time_trusted(true);
    }

    /* ---- interrupt wiring: INTA open-drain, active low ---- */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << RTC_INTA_PIN) | (1ULL << RTC_INTB_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,            /* fallback if the board has no external pull-up */
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    if ((err = pcf2131_alarm_enable_int(s_rtc, true, false)) != ESP_OK) return err;   /* alarm -> INTA */

    if (xTaskCreate(rtc_task, "rtc", 4096, NULL, 6, &s_task) != pdPASS) return ESP_FAIL;

    /* first alarm: next full minute (00:00:00 -> 00:01:00) */
    pcf2131_time_t cur; bool o2;
    pcf2131_get_time(s_rtc, &cur, &o2);
    if ((err = arm_next_minute(pcf2131_to_epoch(&cur))) != ESP_OK) return err;

    gpio_set_intr_type(RTC_INTA_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);                     /* returns ESP_ERR_INVALID_STATE if already installed - fine */
    gpio_isr_handler_add(RTC_INTA_PIN, inta_isr, NULL);

    ESP_LOGI(TAG, "RTC service running (INTA=GPIO%d), first alarm armed", RTC_INTA_PIN);
    return ESP_OK;
}

void rtc_service_on_time_set(void)
{
    if (!s_task) return;
    s_resync_req = true;
    xTaskNotifyGive(s_task);
}
