#include "pcf2131.h"
#include <sys/time.h>
#include "esp_log.h"

static const char *TAG = "pcf2131";
#define I2C_TIMEOUT_MS 100

static uint8_t dec2bcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }
static uint8_t bcd2dec(uint8_t v) { return (uint8_t)(((v >> 4) * 10) + (v & 0x0F)); }

/* days since 1970-01-01 for a civil date (Howard Hinnant's algorithm) */
static int64_t days_from_civil(int y, unsigned m, unsigned d)
{
    y -= m <= 2;
    const int era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

time_t pcf2131_to_epoch(const pcf2131_time_t *t)
{
    return (time_t)(days_from_civil(t->year, t->month, t->day) * 86400
                    + t->hour * 3600 + t->minute * 60 + t->second);
}

void pcf2131_from_epoch(time_t epoch, pcf2131_time_t *t)
{
    struct tm tm;
    gmtime_r(&epoch, &tm);
    t->year    = (uint16_t)(tm.tm_year + 1900);
    t->month   = (uint8_t)(tm.tm_mon + 1);
    t->day     = (uint8_t)tm.tm_mday;
    t->weekday = (uint8_t)tm.tm_wday;
    t->hour    = (uint8_t)tm.tm_hour;
    t->minute  = (uint8_t)tm.tm_min;
    t->second  = (uint8_t)tm.tm_sec;
}

/* ---------------- Bus ---------------- */
esp_err_t pcf2131_init(pcf2131_t *rtc, i2c_master_bus_handle_t bus, uint32_t scl_hz)
{
    if (!rtc || !bus) return ESP_ERR_INVALID_ARG;
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = PCF2131_I2C_ADDR,
        .scl_speed_hz    = scl_hz ? scl_hz : 100000,
    };
    return i2c_master_bus_add_device(bus, &cfg, &rtc->dev);
}

esp_err_t pcf2131_is_present(i2c_master_bus_handle_t bus)
{
    return i2c_master_probe(bus, PCF2131_I2C_ADDR, I2C_TIMEOUT_MS);
}

esp_err_t pcf2131_read_regs(pcf2131_t *rtc, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(rtc->dev, &reg, 1, buf, len, I2C_TIMEOUT_MS);
}

esp_err_t pcf2131_write_regs(pcf2131_t *rtc, uint8_t reg, const uint8_t *buf, size_t len)
{
    uint8_t tmp[16];
    if (len > sizeof(tmp) - 1) return ESP_ERR_INVALID_SIZE;
    tmp[0] = reg;
    for (size_t i = 0; i < len; i++) tmp[i + 1] = buf[i];
    return i2c_master_transmit(rtc->dev, tmp, len + 1, I2C_TIMEOUT_MS);
}

/* ---------------- Time ---------------- */
esp_err_t pcf2131_config_24h(pcf2131_t *rtc)
{
    uint8_t c1;
    esp_err_t err = pcf2131_read_regs(rtc, PCF2131_REG_CTRL1, &c1, 1);
    if (err != ESP_OK) return err;
    if (!(c1 & PCF2131_CTRL1_12_24)) return ESP_OK;
    c1 &= (uint8_t)~PCF2131_CTRL1_12_24;
    return pcf2131_write_regs(rtc, PCF2131_REG_CTRL1, &c1, 1);
}

esp_err_t pcf2131_get_time(pcf2131_t *rtc, pcf2131_time_t *t, bool *osf)
{
    uint8_t r[7];
    esp_err_t err = pcf2131_read_regs(rtc, PCF2131_REG_SECONDS, r, sizeof(r));
    if (err != ESP_OK) return err;

    if (osf) *osf = (r[0] & PCF2131_OSF_BIT) != 0;

    t->second  = bcd2dec(r[0] & 0x7F);
    t->minute  = bcd2dec(r[1] & 0x7F);
    t->hour    = bcd2dec(r[2] & 0x3F);          /* 24 h mode */
    t->day     = bcd2dec(r[3] & 0x3F);
    t->weekday = (uint8_t)(r[4] & 0x07);
    t->month   = bcd2dec(r[5] & 0x1F);
    t->year    = (uint16_t)(2000 + bcd2dec(r[6]));

    if (t->month < 1 || t->month > 12 || t->day < 1 || t->day > 31 ||
        t->hour > 23 || t->minute > 59 || t->second > 59) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t pcf2131_set_time(pcf2131_t *rtc, const pcf2131_time_t *t)
{
    if (t->year < 2000 || t->year > 2099) return ESP_ERR_INVALID_ARG;

    pcf2131_time_t c;                           /* weekday computed from the date */
    pcf2131_from_epoch(pcf2131_to_epoch(t), &c);

    uint8_t r[8] = {
        0x00,                                   /* 100th seconds */
        dec2bcd(c.second),                      /* bit7 = 0 clears OSF */
        dec2bcd(c.minute),
        dec2bcd(c.hour),
        dec2bcd(c.day),
        c.weekday,
        dec2bcd(c.month),
        dec2bcd((uint8_t)(c.year - 2000)),
    };
    return pcf2131_write_regs(rtc, PCF2131_REG_100TH_SEC, r, sizeof(r));
}

/* ---------------- Alarm / interrupt ---------------- */
esp_err_t pcf2131_set_alarm_hm(pcf2131_t *rtc, uint8_t hour, uint8_t minute)
{
    if (hour > 23 || minute > 59) return ESP_ERR_INVALID_ARG;
    uint8_t a[5] = {
        dec2bcd(0),                  /* second = 00, AE_S = 0 (compared) */
        dec2bcd(minute),             /* AE_M = 0 */
        dec2bcd(hour),               /* AE_H = 0 (24 h mode) */
        PCF2131_ALRM_AE,             /* day ignored */
        PCF2131_ALRM_AE,             /* weekday ignored */
    };
    return pcf2131_write_regs(rtc, PCF2131_REG_ALRM_SEC, a, sizeof(a));
}

/* Writing a flag bit as 1 leaves it unchanged; writing 0 clears it (data sheet 7.11.5). */
esp_err_t pcf2131_alarm_enable_int(pcf2131_t *rtc, bool inta, bool intb)
{
    uint8_t c2 = (uint8_t)(PCF2131_CTRL2_MSF | PCF2131_CTRL2_AF | PCF2131_CTRL2_AIE); /* WDTF = 0 */
    esp_err_t err = pcf2131_write_regs(rtc, PCF2131_REG_CTRL2, &c2, 1);
    if (err != ESP_OK) return err;

    const struct { uint8_t reg; bool route; } m[2] = {
        { PCF2131_REG_INTA_MASK1, inta }, { PCF2131_REG_INTB_MASK1, intb } };
    for (int i = 0; i < 2; i++) {
        uint8_t mask;
        if ((err = pcf2131_read_regs(rtc, m[i].reg, &mask, 1)) != ESP_OK) return err;
        mask &= 0x3F;                                   /* bits 7:6 are unused */
        if (m[i].route) mask &= (uint8_t)~PCF2131_MASK1_AIE;   /* 0 = alarm reaches the pin */
        else            mask |= PCF2131_MASK1_AIE;
        if ((err = pcf2131_write_regs(rtc, m[i].reg, &mask, 1)) != ESP_OK) return err;
    }
    return ESP_OK;
}

esp_err_t pcf2131_alarm_clear_flag(pcf2131_t *rtc)
{
    uint8_t c2;
    esp_err_t err = pcf2131_read_regs(rtc, PCF2131_REG_CTRL2, &c2, 1);
    if (err != ESP_OK) return err;
    /* keep AIE, leave MSF unchanged (write 1), WDTF = 0, AF = 0 -> clears only AF */
    c2 = (uint8_t)(PCF2131_CTRL2_MSF | (c2 & PCF2131_CTRL2_AIE));
    return pcf2131_write_regs(rtc, PCF2131_REG_CTRL2, &c2, 1);
}

esp_err_t pcf2131_alarm_flag(pcf2131_t *rtc, bool *af)
{
    uint8_t c2;
    esp_err_t err = pcf2131_read_regs(rtc, PCF2131_REG_CTRL2, &c2, 1);
    if (err == ESP_OK) *af = (c2 & PCF2131_CTRL2_AF) != 0;
    return err;
}

/* ---------------- Convenience ---------------- */
esp_err_t pcf2131_load_system_time(pcf2131_t *rtc)
{
    pcf2131_time_t t;
    bool osf = false;
    esp_err_t err = pcf2131_get_time(rtc, &t, &osf);
    if (err != ESP_OK) return err;
    if (osf || t.year < 2025) {
        ESP_LOGW(TAG, "RTC time not valid (osf=%d year=%u)", osf, t.year);
        return ESP_ERR_INVALID_STATE;
    }
    struct timeval tv = { .tv_sec = pcf2131_to_epoch(&t), .tv_usec = 0 };
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "System time set from RTC: %04u-%02u-%02u %02u:%02u:%02u UTC",
             t.year, t.month, t.day, t.hour, t.minute, t.second);
    return ESP_OK;
}

esp_err_t pcf2131_store_system_time(pcf2131_t *rtc)
{
    pcf2131_time_t t;
    pcf2131_from_epoch(time(NULL), &t);
    return pcf2131_set_time(rtc, &t);
}
