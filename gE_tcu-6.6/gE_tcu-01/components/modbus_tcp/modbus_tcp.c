#include "modbus_tcp.h"
#include <string.h>
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "MB_TCP";

#ifndef MODBUS_TCP_PORT
#define MODBUS_TCP_PORT 502
#endif
#define MB_REG_MODE    0
#define MB_REG_STATE   1
#define MB_REG_CUR     2
#define MB_REG_TARGET  3
#define MB_REG_STATUS  4
#define MB_REG_VERSION 5
#define MB_REG_WIND    6
#define MB_REGMAP_VERSION 2

#define MB_EXC_ILLEGAL_FUNCTION  0x01
#define MB_EXC_ILLEGAL_ADDRESS   0x02
#define MB_FC_READ_HOLDING  0x03
#define MB_FC_WRITE_SINGLE  0x06

static uint16_t                   s_regs[MODBUS_REG_COUNT];
static SemaphoreHandle_t          s_mutex;
static modbus_tcp_mode_write_cb_t s_mode_cb;

/* TCP is a byte stream: recv() may return fewer bytes than asked, so loop until we have them all. */
static bool read_exact(int sock, uint8_t *buf, int len)
{
    int got = 0;
    while (got < len) {
        int n = recv(sock, buf + got, len - got, 0);
        if (n <= 0) return false;            /* closed or error */
        got += n;
    }
    return true;
}

static void handle_client(int sock)
{
    uint8_t req[260];
    for (;;) {
        /* MBAP header (7 bytes): txn_id(2) proto_id(2) length(2) unit_id(1) */
        if (!read_exact(sock, req, 7)) return;       /* client closed / error */

        uint16_t txn_id   = (req[0] << 8) | req[1];
        uint16_t proto_id = (req[2] << 8) | req[3];
        uint16_t length   = (req[4] << 8) | req[5];
        uint8_t  unit_id  = req[6];
        if (proto_id != 0 || length < 2 || length > sizeof(req) - 6) return;   /* not Modbus TCP: drop the client */

        int pdu_len = length - 1;                     /* minus unit_id already read */
        if (!read_exact(sock, req + 7, pdu_len)) return;

        uint8_t fc = req[7];
        uint8_t resp[7 + 2 + 2 * MODBUS_REG_COUNT + 8];   /* MBAP(7) + FC3 header(2) + data, with spare */
        int resp_len = 0;

        if (fc == MB_FC_READ_HOLDING && pdu_len == 5) {
            uint16_t addr = (req[8] << 8) | req[9];
            uint16_t qty  = (req[10] << 8) | req[11];
            if (qty == 0 || qty > 16 || (uint32_t)addr + qty > MODBUS_REG_COUNT) {
                resp[7] = fc | 0x80; resp[8] = MB_EXC_ILLEGAL_ADDRESS; resp_len = 2;
            } else {
                resp[7] = fc; resp[8] = (uint8_t)(qty * 2);
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                for (int i = 0; i < qty; i++) {
                    resp[9 + 2 * i]     = (uint8_t)(s_regs[addr + i] >> 8);
                    resp[9 + 2 * i + 1] = (uint8_t)(s_regs[addr + i] & 0xFF);
                }
                xSemaphoreGive(s_mutex);
                resp_len = 2 + qty * 2;
            }
        } else if (fc == MB_FC_WRITE_SINGLE && pdu_len == 5) {
            uint16_t addr = (req[8] << 8) | req[9];
            uint16_t val  = (req[10] << 8) | req[11];
            if (addr != MB_REG_MODE) {                 /* only the mode register is writable */
                resp[7] = fc | 0x80; resp[8] = MB_EXC_ILLEGAL_ADDRESS; resp_len = 2;
            } else {
                xSemaphoreTake(s_mutex, portMAX_DELAY);
                s_regs[MB_REG_MODE] = val ? 1 : 0;
                xSemaphoreGive(s_mutex);
                if (s_mode_cb) s_mode_cb(val != 0);
                memcpy(&resp[7], &req[7], 5);           /* FC6 echoes the request */
                resp_len = 5;
            }
        } else {
            resp[7] = fc | 0x80; resp[8] = MB_EXC_ILLEGAL_FUNCTION; resp_len = 2;
        }

        uint16_t out_len = 1 + resp_len;                /* unit_id + pdu */
        resp[0] = (uint8_t)(txn_id >> 8); resp[1] = (uint8_t)(txn_id & 0xFF);
        resp[2] = 0; resp[3] = 0;                        /* protocol id */
        resp[4] = (uint8_t)(out_len >> 8); resp[5] = (uint8_t)(out_len & 0xFF);
        resp[6] = unit_id;
        send(sock, resp, 7 + resp_len, 0);
    }
}

static void server_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) { ESP_LOGE(TAG, "socket() failed: errno %d", errno); vTaskDelete(NULL); }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_addr.s_addr = htonl(INADDR_ANY),
                                 .sin_port = htons(MODBUS_TCP_PORT) };
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "bind() failed: errno %d", errno); close(listen_sock); vTaskDelete(NULL);
    }
    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "listen() failed: errno %d", errno); close(listen_sock); vTaskDelete(NULL);
    }
    ESP_LOGI(TAG, "Modbus TCP slave listening on port %d (unit id: any, FC03/FC06 only)", MODBUS_TCP_PORT);

    for (;;) {
        struct sockaddr_in src; socklen_t src_len = sizeof(src);
        int sock = accept(listen_sock, (struct sockaddr *)&src, &src_len);
        if (sock < 0) { vTaskDelay(pdMS_TO_TICKS(1000)); continue; }

        char ip[16]; inet_ntoa_r(src.sin_addr, ip, sizeof(ip));
        ESP_LOGI(TAG, "SCADA client connected: %s", ip);
        handle_client(sock);
        ESP_LOGI(TAG, "SCADA client disconnected: %s", ip);
        close(sock);
    }
}

esp_err_t modbus_tcp_start(modbus_tcp_mode_write_cb_t cb)
{
    s_mode_cb = cb;
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;
    s_regs[MB_REG_VERSION] = MB_REGMAP_VERSION;
    return xTaskCreate(server_task, "mb_tcp", 4096, NULL, 4, NULL) == pdPASS ? ESP_OK : ESP_FAIL;
}

void modbus_tcp_update_regs(bool manual_mode, float current_tilt_deg, float target_tilt_deg,
                            int tracker_state, bool motor_fault, bool inclinometer_invalid,
                            bool rtc_untrusted, float wind_speed_ms, bool anemometer_offline)
{
    if (!s_mutex) return;
    uint16_t status = (motor_fault ? 1 : 0) | (inclinometer_invalid ? 2 : 0) |
                      (rtc_untrusted ? 4 : 0) | (anemometer_offline ? 16 : 0);
#ifdef ETH_MISO_PIN                                      /* eth_w5500 component is in this build */
    extern bool eth_w5500_link_is_up(void);
    if (eth_w5500_link_is_up()) status |= 8;
#endif
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_regs[MB_REG_MODE]   = manual_mode ? 1 : 0;          /* reflects the actual tracker mode */
    s_regs[MB_REG_STATE]  = (uint16_t)tracker_state;
    s_regs[MB_REG_CUR]    = (uint16_t)(int16_t)(current_tilt_deg * 10.0f);
    s_regs[MB_REG_TARGET] = (uint16_t)(int16_t)(target_tilt_deg * 10.0f);
    s_regs[MB_REG_STATUS] = status;
    s_regs[MB_REG_WIND]   = anemometer_offline ? 0 : (uint16_t)(wind_speed_ms * 10.0f + 0.5f);
    xSemaphoreGive(s_mutex);
}
