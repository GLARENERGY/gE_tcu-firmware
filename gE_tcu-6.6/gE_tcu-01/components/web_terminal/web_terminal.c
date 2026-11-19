#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "web_terminal.h"
#include "terminal_page.h"

#define RING_SIZE      16384      /* bytes of log history kept in RAM */
#define CHUNK_MAX      6000       /* max bytes returned per poll */
#define LINE_MAX_LEN   256

static char              s_ring[RING_SIZE];
static uint32_t          s_total;                 /* total bytes ever written (monotonic) */
static SemaphoreHandle_t s_lock;
static vprintf_like_t    s_prev_vprintf;
static void            (*s_activity_cb)(void);

/* esp_log hook: keep a copy for the web terminal, then pass the line on to the normal console output. */
static int log_hook(const char *fmt, va_list args)
{
    int ret;
    if (s_lock) {
        char line[LINE_MAX_LEN];
        va_list copy;
        va_copy(copy, args);
        int n = vsnprintf(line, sizeof(line), fmt, copy);
        va_end(copy);
        if (n > 0) {
            if (n >= (int)sizeof(line)) n = sizeof(line) - 1;
            if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
                for (int i = 0; i < n; i++) s_ring[(s_total + i) % RING_SIZE] = line[i];
                s_total += (uint32_t)n;
                xSemaphoreGive(s_lock);
            }
        }
    }
    ret = s_prev_vprintf ? s_prev_vprintf(fmt, args) : vprintf(fmt, args);
    return ret;
}

void web_terminal_init(void)
{
    if (s_lock) return;
    s_lock = xSemaphoreCreateMutex();
    s_prev_vprintf = esp_log_set_vprintf(log_hook);
}

/* ---- HTTP ---- */
static esp_err_t page_handler(httpd_req_t *req)
{
    if (s_activity_cb) s_activity_cb();
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, TERMINAL_HTML, TERMINAL_HTML_LEN);
}

static esp_err_t log_handler(httpd_req_t *req)
{
    if (s_activity_cb) s_activity_cb();

    uint32_t from = 0;
    char q[48], v[16];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK &&
        httpd_query_key_value(q, "from", v, sizeof(v)) == ESP_OK)
        from = (uint32_t)strtoul(v, NULL, 10);

    char *buf = malloc(CHUNK_MAX);
    if (!buf) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory"); return ESP_OK; }

    uint32_t total = 0, start = 0, len = 0;
    bool reset = false;
    if (s_lock && xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        total = s_total;
        uint32_t oldest = (total > RING_SIZE) ? total - RING_SIZE : 0;
        start = from;
        if (start > total) { start = oldest; reset = (from != 0); }   /* device rebooted since the last poll */
        if (start < oldest) start = oldest;                           /* fell too far behind: skip ahead */
        len = total - start;
        if (len > CHUNK_MAX) len = CHUNK_MAX;
        for (uint32_t i = 0; i < len; i++) buf[i] = s_ring[(start + i) % RING_SIZE];
        xSemaphoreGive(s_lock);
    }

    char next[16];
    snprintf(next, sizeof(next), "%lu", (unsigned long)(start + len));
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_set_hdr(req, "X-Next", next);
    httpd_resp_set_hdr(req, "X-Reset", reset ? "1" : "0");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, len);
    free(buf);
    return ESP_OK;
}

void web_terminal_register(httpd_handle_t server, void (*activity_cb)(void))
{
    s_activity_cb = activity_cb;
    httpd_uri_t p1 = { .uri = "/terminal",     .method = HTTP_GET, .handler = page_handler, .user_ctx = NULL };
    httpd_uri_t p2 = { .uri = "/terminal/",    .method = HTTP_GET, .handler = page_handler, .user_ctx = NULL };
    httpd_uri_t lg = { .uri = "/terminal/log", .method = HTTP_GET, .handler = log_handler,  .user_ctx = NULL };
    httpd_register_uri_handler(server, &p1);
    httpd_register_uri_handler(server, &p2);
    httpd_register_uri_handler(server, &lg);
}
