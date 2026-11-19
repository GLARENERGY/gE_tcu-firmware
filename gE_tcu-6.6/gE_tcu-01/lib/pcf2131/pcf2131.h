#pragma once
/*
 * PCF2131 RTC driver for ESP-IDF >= 5.2 (new i2c_master API).
 * Register map and flag rules follow NXP PCF2131 data sheet Rev. 2.5.
 * The driver attaches to an EXISTING bus handle so the LIS2DW12 can share it.
 *
 * Alarm model used by this driver: "daily alarm at HH:MM:00" (second + minute + hour
 * compared, day and weekday ignored). The TCU re-arms it every minute for the next minute.
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PCF2131_I2C_ADDR        0x53   /* 7-bit (1010 011) */

/* Registers */
#define PCF2131_REG_CTRL1       0x00
#define PCF2131_REG_CTRL2       0x01
#define PCF2131_REG_CTRL3       0x02
#define PCF2131_REG_SR_RESET    0x05
#define PCF2131_REG_100TH_SEC   0x06
#define PCF2131_REG_SECONDS     0x07   /* bit7 = OSF */
#define PCF2131_REG_MINUTES     0x08
#define PCF2131_REG_HOURS       0x09
#define PCF2131_REG_DAYS        0x0A
#define PCF2131_REG_WEEKDAYS    0x0B
#define PCF2131_REG_MONTHS      0x0C
#define PCF2131_REG_YEARS       0x0D
#define PCF2131_REG_ALRM_SEC    0x0E   /* bit7 = AE_S (1 = ignore) */
#define PCF2131_REG_ALRM_MIN    0x0F
#define PCF2131_REG_ALRM_HOUR   0x10
#define PCF2131_REG_ALRM_DAY    0x11
#define PCF2131_REG_ALRM_WDAY   0x12
#define PCF2131_REG_INTA_MASK1  0x31   /* bit2 AIEA: 0 = alarm drives INTA */
#define PCF2131_REG_INTA_MASK2  0x32
#define PCF2131_REG_INTB_MASK1  0x33   /* bit2 AIEB: 0 = alarm drives INTB */
#define PCF2131_REG_INTB_MASK2  0x34

/* Bits */
#define PCF2131_OSF_BIT         0x80   /* Seconds.7  - clock integrity NOT guaranteed */
#define PCF2131_CTRL1_12_24     0x04   /* 0 = 24 h mode */
#define PCF2131_CTRL2_MSF       0x80
#define PCF2131_CTRL2_WDTF      0x40
#define PCF2131_CTRL2_AF        0x10   /* alarm flag */
#define PCF2131_CTRL2_AIE       0x02   /* alarm interrupt enable */
#define PCF2131_ALRM_AE         0x80   /* alarm enable bit: 1 = field ignored */
#define PCF2131_MASK1_AIE       0x04   /* INT_x_MASK1 bit2 */

typedef struct {
    uint16_t year;     /* full year, 2000..2099 */
    uint8_t  month;    /* 1-12 */
    uint8_t  day;      /* 1-31 */
    uint8_t  weekday;  /* 0 = Sunday .. 6 = Saturday (computed from the date on write) */
    uint8_t  hour;     /* 0-23 */
    uint8_t  minute;   /* 0-59 */
    uint8_t  second;   /* 0-59 */
} pcf2131_time_t;

typedef struct {
    i2c_master_dev_handle_t dev;
} pcf2131_t;

/* ---- Bus / presence ---- */
esp_err_t pcf2131_init(pcf2131_t *rtc, i2c_master_bus_handle_t bus, uint32_t scl_hz);
esp_err_t pcf2131_is_present(i2c_master_bus_handle_t bus);

/* ---- Raw register access ---- */
esp_err_t pcf2131_read_regs(pcf2131_t *rtc, uint8_t reg, uint8_t *buf, size_t len);
esp_err_t pcf2131_write_regs(pcf2131_t *rtc, uint8_t reg, const uint8_t *buf, size_t len);

/* ---- Time ---- */
/* Forces 24 h mode. */
esp_err_t pcf2131_config_24h(pcf2131_t *rtc);
/* osf (optional) = true when the oscillator-stop flag is set (time NOT trustworthy). */
esp_err_t pcf2131_get_time(pcf2131_t *rtc, pcf2131_time_t *t, bool *osf);
/* Writes 100th-seconds=0 + all time registers in one burst; this also clears OSF. */
esp_err_t pcf2131_set_time(pcf2131_t *rtc, const pcf2131_time_t *t);

/* ---- Alarm / interrupt ---- */
/* Alarm at hour:minute:00 (day/weekday ignored). Does NOT clear AF or enable the interrupt. */
esp_err_t pcf2131_set_alarm_hm(pcf2131_t *rtc, uint8_t hour, uint8_t minute);
/* Set AIE and route the alarm to INTA and/or INTB (open-drain, active LOW; INT follows AF). */
esp_err_t pcf2131_alarm_enable_int(pcf2131_t *rtc, bool inta, bool intb);
/* Clear only AF (releases the INT pin). Other flags are left untouched (data sheet 7.11.5). */
esp_err_t pcf2131_alarm_clear_flag(pcf2131_t *rtc);
esp_err_t pcf2131_alarm_flag(pcf2131_t *rtc, bool *af);

/* ---- Conversions (UTC, no timezone) ---- */
time_t pcf2131_to_epoch(const pcf2131_time_t *t);
void   pcf2131_from_epoch(time_t epoch, pcf2131_time_t *t);

/* ---- Convenience ---- */
esp_err_t pcf2131_load_system_time(pcf2131_t *rtc);   /* RTC -> system clock (ESP_ERR_INVALID_STATE if OSF/invalid) */
esp_err_t pcf2131_store_system_time(pcf2131_t *rtc);  /* system clock -> RTC */

#ifdef __cplusplus
}
#endif
