#include "eth_w5500.h"
#include "AppConfig.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
/* ESP-IDF 6.0 moved the W5500 driver out of core into the registry component espressif/w5500 */
#include "esp_eth_mac_w5500.h"
#include "esp_eth_phy_w5500.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "ETH";

/* ESP32-C6 has a single general-purpose SPI controller (SPI2/GPSPI2) free for
 * peripherals - fixed here rather than in config.yaml, since config.yaml values become
 * C #define text and an enum name would get wrapped in quotes by the generator. */
#define ETH_SPI_HOST SPI2_HOST

#ifndef ETH_SPI_CLOCK_MHZ
#define ETH_SPI_CLOCK_MHZ 8      /* W5500 allows ~33 MHz, but jumper wires/breadboards corrupt frames well
                                  * before that (link can show "up" while packets are lost). Raise it later. */
#endif
#ifndef ETH_PHY_ADDR
#define ETH_PHY_ADDR 1           /* W5500's fixed internal PHY address */
#endif

static esp_netif_t *s_netif;
static volatile bool s_link_up;
static volatile bool s_has_ip;

#ifndef ETH_USE_STATIC_IP
/* DHCP with a static fallback: if no DHCP server answers (e.g. the cable goes straight to a PC),
 * the TCU takes a fixed address so the website / Modbus are still reachable. Override any of these
 * in config.yaml (others:). Set your PC to the same subnet, e.g. 192.168.1.10 / 255.255.255.0. */
#ifndef ETH_FALLBACK_IP
#define ETH_FALLBACK_IP        "192.168.1.50"
#endif
#ifndef ETH_FALLBACK_GW
#define ETH_FALLBACK_GW        "192.168.1.1"
#endif
#ifndef ETH_FALLBACK_NETMASK
#define ETH_FALLBACK_NETMASK   "255.255.255.0"
#endif
#ifndef ETH_FALLBACK_DELAY_S
#define ETH_FALLBACK_DELAY_S   10
#endif

static esp_timer_handle_t s_fallback_timer;

static void fallback_timer_cb(void *arg)
{
    if (s_has_ip || !s_netif) return;
    ESP_LOGW(TAG, "No DHCP reply after %d s - using fallback static IP %s", ETH_FALLBACK_DELAY_S, ETH_FALLBACK_IP);
    esp_netif_dhcpc_stop(s_netif);
    esp_netif_ip_info_t ip = { 0 };
    ip.ip.addr      = esp_ip4addr_aton(ETH_FALLBACK_IP);
    ip.gw.addr      = esp_ip4addr_aton(ETH_FALLBACK_GW);
    ip.netmask.addr = esp_ip4addr_aton(ETH_FALLBACK_NETMASK);
    esp_netif_set_ip_info(s_netif, &ip);
    ESP_LOGI(TAG, "Set your PC to 192.168.1.x / %s, then open http://%s", ETH_FALLBACK_NETMASK, ETH_FALLBACK_IP);
}

static void start_fallback_timer(void)
{
    if (s_has_ip) return;
    esp_netif_dhcpc_start(s_netif);                 /* no-op if already running; restarts it after a fallback */
    if (!s_fallback_timer) {
        const esp_timer_create_args_t args = { .callback = fallback_timer_cb, .name = "eth_fallback" };
        if (esp_timer_create(&args, &s_fallback_timer) != ESP_OK) return;
    }
    esp_timer_stop(s_fallback_timer);               /* error if not running - ignored */
    esp_timer_start_once(s_fallback_timer, (uint64_t)ETH_FALLBACK_DELAY_S * 1000000ULL);
}
#endif

static void on_eth_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        s_link_up = true;
#ifdef ETH_USE_STATIC_IP
        ESP_LOGI(TAG, "Link up (static IP %s)", ETH_STATIC_IP);
#else
        ESP_LOGI(TAG, "Link up - waiting for DHCP (falls back to %s after %d s)", ETH_FALLBACK_IP, ETH_FALLBACK_DELAY_S);
        start_fallback_timer();
#endif
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        s_link_up = false;
        s_has_ip = false;
        ESP_LOGW(TAG, "Link down - check the cable");
        break;
    case ETHERNET_EVENT_START:        ESP_LOGI(TAG, "Driver started"); break;
    case ETHERNET_EVENT_STOP:         ESP_LOGI(TAG, "Driver stopped"); break;
    default: break;
    }
}

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    s_has_ip = true;
#ifndef ETH_USE_STATIC_IP
    if (s_fallback_timer) esp_timer_stop(s_fallback_timer);
#endif
    ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
    ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&evt->ip_info.ip));
}

esp_netif_t *eth_w5500_start(void)
{
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "esp_netif_init: %s", esp_err_to_name(err)); return NULL; }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "event loop: %s", esp_err_to_name(err)); return NULL; }

    /* The W5500 INT line is attached with gpio_isr_handler_add(); on ESP-IDF 6 the driver no longer
     * installs the GPIO ISR service itself. Without it the log shows "GPIO isr service is not
     * installed" and the link never receives packets. ESP_ERR_INVALID_STATE = already installed (fine). */
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) { ESP_LOGE(TAG, "gpio_install_isr_service: %s", esp_err_to_name(err)); return NULL; }

    spi_bus_config_t bus = {
        .miso_io_num = ETH_MISO_PIN,
        .mosi_io_num = ETH_MOSI_PIN,
        .sclk_io_num = ETH_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    if ((err = spi_bus_initialize(ETH_SPI_HOST, &bus, SPI_DMA_CH_AUTO)) != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return NULL;
    }

    spi_device_interface_config_t dev = {
        .command_bits = 16,
        .address_bits = 8,
        .mode = 0,
        .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = ETH_CS_PIN,
        .queue_size = 20,
    };

    eth_w5500_config_t w5500_cfg = ETH_W5500_DEFAULT_CONFIG(ETH_SPI_HOST, &dev);
#ifdef ETH_USE_INT_PIN
    w5500_cfg.int_gpio_num = ETH_INT_PIN;            /* RX is signalled by the W5500 INT line */
    ESP_LOGI(TAG, "RX mode: interrupt on GPIO %d", ETH_INT_PIN);
#else
    /* Polling mode: the driver checks for received frames every few ms over SPI, so a missing or
     * loose INT wire cannot stop RX (symptom: the TCU transmits but never answers ARP/ping).
     * Define ETH_USE_INT_PIN (config.yaml: others: ETH_USE_INT_PIN: true) once INT is verified. */
    w5500_cfg.int_gpio_num = -1;
    w5500_cfg.poll_period_ms = 10;
    ESP_LOGI(TAG, "RX mode: polling every 10 ms (INT pin not used)");
#endif

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_cfg, &mac_cfg);
    if (!mac) { ESP_LOGE(TAG, "esp_eth_mac_new_w5500 failed"); return NULL; }

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = -1;                 /* no reset pin wired; set ETH_RST_PIN in config.yaml if yours has one */
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_cfg);
    if (!phy) { ESP_LOGE(TAG, "esp_eth_phy_new_w5500 failed"); return NULL; }

    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    if ((err = esp_eth_driver_install(&eth_cfg, &eth_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_driver_install: %s", esp_err_to_name(err));
        return NULL;
    }

    /* Locally-administered MAC (bit 1 of the first byte set) so it never collides with a
     * real vendor-assigned address. Change if your SCADA network requires a specific MAC. */
    uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x54, 0x43, 0x55};   /* "TCU" in the low 3 bytes */
    esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr);

    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_netif = esp_netif_new(&netif_cfg);
    if (!s_netif) { ESP_LOGE(TAG, "esp_netif_new failed"); return NULL; }

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
    esp_netif_attach(s_netif, glue);

#ifdef ETH_USE_STATIC_IP
    esp_netif_dhcpc_stop(s_netif);
    esp_netif_ip_info_t ip_info = { 0 };
    ip_info.ip.addr      = esp_ip4addr_aton(ETH_STATIC_IP);
    ip_info.gw.addr      = esp_ip4addr_aton(ETH_STATIC_GW);
    ip_info.netmask.addr = esp_ip4addr_aton(ETH_STATIC_NETMASK);
    esp_netif_set_ip_info(s_netif, &ip_info);
    ESP_LOGI(TAG, "Static IP: %s", ETH_STATIC_IP);
#endif

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &on_eth_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &on_got_ip, NULL));

    if ((err = esp_eth_start(eth_handle)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start: %s", esp_err_to_name(err));
        return NULL;
    }

    ESP_LOGI(TAG, "W5500 started (MISO=%d MOSI=%d SCLK=%d CS=%d INT=%d)",
             ETH_MISO_PIN, ETH_MOSI_PIN, ETH_SCLK_PIN, ETH_CS_PIN, ETH_INT_PIN);
    return s_netif;
}

bool eth_w5500_link_is_up(void) { return s_link_up; }
