/*
 * Glarenergy TCU - application skeleton
 *
 *  Start -> NVS -> Data retrieve --yes--> Master byte --1--> Inclinometer (LIS2DW12) task --\
 *                       |                      \--0--> Legacy RS485 sensor task ------------+--> RTC task
 *                       no                                                                  +--> Tracker task
 *                       v
 *                 BLE commissioning task -> Check ok -> (save + restart = back to Start)
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c_master.h"

#include "AppConfig.h"                 /* generated from config.yaml (pins, tracker settings) */
#include "tcu_board_hw_interface.h"
#include "hal_tcu.h"
#include "inclinometer.h"
#include "anemometer.h"
#include "solar_math.h"
#include "tracker_core.h"
#include "web_ui.h"
#include "eth_w5500.h"
#include "modbus_tcp.h"
#include "rtc_service.h"
#include "status_led.h"
#include "web_terminal.h"

static const char *TAG = "TCU";

#define I2C_FREQ_HZ  100000

/* TESTING ONLY: until BLE commissioning exists, write commissioned=1 / master_byte=1
 * to NVS when they are missing. Set to 0 for production. */
#define DEV_AUTO_COMMISSION   1
#define DEV_DEFAULT_MASTER    1

/* ---- NVS ---- */
#define NVS_NS              "tracker_cfg"
#define NVS_KEY_COMMISSION  "commissioned"
#define NVS_KEY_MASTER      "master_byte"

#define MASTER_INCLINOMETER 0          /* legacy RS485 sensor */
#define MASTER_ACCEL        1          /* LIS2DW12 */

/* ---- Shared state ---- */
static i2c_master_bus_handle_t g_i2c_bus;
static QueueHandle_t           g_tilt_queue;    /* tilt_msg_t, length 1 */
tracker_handle_t               tracker;         /* also used by the web handlers later */

/* SCADA (Modbus TCP) writes HR0 (mode) -> this runs in the mb_tcp task, so go through
 * the same thread-safe path the web UI uses. */
static void on_scada_mode_write(bool manual)
{
    if (tracker) tracker_set_mode(tracker, manual ? TRACKER_MODE_MANUAL : TRACKER_MODE_AUTO);
}

/* ================= NVS / config ================= */
typedef enum { CFG_OK, CFG_MISSING, CFG_INVALID, CFG_ERROR } cfg_status_t;

static cfg_status_t config_load(uint8_t *master_byte)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND) return CFG_MISSING;
    if (err != ESP_OK) { ESP_LOGE(TAG, "nvs_open: %s", esp_err_to_name(err)); return CFG_ERROR; }

    uint8_t commissioned = 0, mb = 0xFF;
    err = nvs_get_u8(h, NVS_KEY_COMMISSION, &commissioned);
    if (err == ESP_OK) err = nvs_get_u8(h, NVS_KEY_MASTER, &mb);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) return CFG_MISSING;
    if (err != ESP_OK)                return CFG_ERROR;
    if (!commissioned)                return CFG_MISSING;
    if (mb != MASTER_ACCEL && mb != MASTER_INCLINOMETER) return CFG_INVALID;

    *master_byte = mb;
    return CFG_OK;
}

static esp_err_t config_save(uint8_t master_byte)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    nvs_set_u8(h, NVS_KEY_MASTER, master_byte);
    nvs_set_u8(h, NVS_KEY_COMMISSION, 1);
    err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* "Check ok": validate, save, restart (restart == loop back to Start) */
static void commissioning_finish(uint8_t master_byte)
{
    if (master_byte != MASTER_ACCEL && master_byte != MASTER_INCLINOMETER) {
        ESP_LOGE(TAG, "Check failed: bad master byte %u", master_byte);
        return;
    }
    if (config_save(master_byte) == ESP_OK) {
        ESP_LOGI(TAG, "Commissioning saved, restarting");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    }
}

/* ================= Tasks ================= */
static void ble_commissioning_task(void *arg)
{
    ESP_LOGW(TAG, "Not commissioned -> BLE commissioning");
    /* TODO: BLE advertising + GATT; when a valid config arrives:
     *       commissioning_finish(received_master_byte);              */
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

static void legacy_rs485_task(void *arg)       /* master byte = 0 */
{
    ESP_LOGI(TAG, "Legacy RS485 inclinometer task (not implemented)");
    /* TODO: modbus RTU read -> tilt_msg_t -> xQueueOverwrite(g_tilt_queue, &m) */
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}

/* Runs tracker_update() at 10 Hz, as the old main loop did. */
static void tracker_task(void *arg)
{
    uint32_t last_log = 0;
    for (;;) {
        tracker_update(tracker);

        uint32_t now = hal_get_millis();
        if (now - last_log >= 2000) {              /* heartbeat: what the tracker sees */
            last_log = now;
            log_restart();                         /* writes the restart line once time is valid */
            inclinometer_data_t inc;
            if (hal_inclinometer_read(&inc))
                ESP_LOGI(TAG, "inc x=%.2f y=%.2f z=%.2f (axis %d) state=%d",
                         (double)inc.x, (double)inc.y, (double)inc.z,
                         INCLINOMETER_AXIS, (int)tracker_get_state(tracker));
            else
                ESP_LOGW(TAG, "inclinometer invalid/stale -> motor held stopped");

            float cur_tilt, target_tilt; tracker_state_t t_state; tracker_mode_t t_mode;
            bool fault, inc_ok;
            tracker_get_snapshot(tracker, &cur_tilt, &target_tilt, &t_state, &t_mode, &fault, &inc_ok);
            float wind = 0.0f;
            bool wind_ok = anemometer_get(&wind);
            if (wind_ok) ESP_LOGI(TAG, "wind %.1f m/s", (double)wind);
            modbus_tcp_update_regs(t_mode == TRACKER_MODE_MANUAL,
                                   cur_tilt, target_tilt, (int)t_state,
                                   fault, !inc_ok, !hal_time_is_trusted(),
                                   wind, !wind_ok);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Runs in the RTC task once per minute (after the RTC alarm interrupt). */
static void on_rtc_minute(time_t minute_epoch, bool trusted)
{
    if (trusted && tracker) tracker_on_minute(tracker, minute_epoch);   /* starts a move at each SolTrack table slot */
}

/* ================= Hardware init ================= */
static esp_err_t i2c_bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,                     /* from config.yaml */
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &g_i2c_bus);
}

/* ================= Entry ================= */
void app_main(void)
{
    web_terminal_init();                                 /* mirror ESP_LOG output to http://glarenergy.local/terminal */
    ESP_LOGI(TAG, "Glarenergy TCU boot");

    /* NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Hardware shared by every branch */
    ESP_ERROR_CHECK(i2c_bus_init());
    g_tilt_queue = xQueueCreate(1, sizeof(tilt_msg_t));
    hal_tcu_attach(g_i2c_bus, g_tilt_queue);

    /* Data retrieve */
    uint8_t master_byte = 0;
    cfg_status_t st = config_load(&master_byte);

#if DEV_AUTO_COMMISSION
    if (st == CFG_MISSING) {
        ESP_LOGW(TAG, "DEV: auto-commissioning with master byte %d", DEV_DEFAULT_MASTER);
        if (config_save(DEV_DEFAULT_MASTER) == ESP_OK) st = config_load(&master_byte);
    }
#endif

    if (st != CFG_OK) {                                  /* "no" branch */
        ESP_LOGW(TAG, "Config status %d -> commissioning", st);
        xTaskCreate(ble_commissioning_task, "ble_comm", 4096, NULL, 5, NULL);
        return;
    }

    /* "yes" branch: bring up the application hardware + tracker */
    hal_ui_init();
    hal_rtc_init();                                      /* finds the PCF2131 (time is handled by rtc_service_start) */
    hal_motor_init();                                    /* outputs forced low */
    solar_math_init();
    tracker = tracker_init();
    if (!tracker) { ESP_LOGE(TAG, "tracker_init failed"); return; }

    /* Master byte -> sensor task */
    if (master_byte == MASTER_ACCEL) {
        ESP_ERROR_CHECK(inclinometer_start(g_i2c_bus, g_tilt_queue));
    } else {
        xTaskCreate(legacy_rs485_task, "rs485", 4096, NULL, 5, NULL);
    }

    /* Anemometer (RS485 Modbus RTU on UART1). For now it is assumed present whatever the master byte is. */
    if (anemometer_start() != ESP_OK)
        ESP_LOGE(TAG, "Anemometer failed to start - wind speed will show as offline");

    /* Both branches converge on the RTC task: PCF2131 minute alarm -> INTA -> ISR -> RTC task */
    if (rtc_service_start(on_rtc_minute) != ESP_OK)
        ESP_LOGE(TAG, "RTC service not running: no minute ticks (tracker falls back to system time)");

    /* Ethernet + Modbus TCP: for the SCADA link. Runs alongside the Wi-Fi AP (web_ui) -
     * different peripherals, no conflict. A missing/unwired W5500 just disables this
     * channel; everything else keeps working. */
    if (eth_w5500_start() == NULL)
        ESP_LOGW(TAG, "W5500 not started - SCADA/Modbus TCP channel unavailable");
    else if (modbus_tcp_start(on_scada_mode_write) != ESP_OK)
        ESP_LOGE(TAG, "Modbus TCP slave failed to start");

    xTaskCreate(tracker_task, "tracker", 8192, NULL, 5, NULL);

    if (status_led_start(tracker, web_ui_client_connected) != ESP_OK)
        ESP_LOGW(TAG, "Status LED not running");

    web_ui_start();                                      /* Wi-Fi AP "Glarenergy-TCU", mDNS, HTTP, SNTP, SPIFFS */
}
