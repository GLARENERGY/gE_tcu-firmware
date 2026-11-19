#pragma once
/* W5500 SPI Ethernet bring-up (ESP-IDF esp_eth driver). */
#include <stdbool.h>
#include "esp_err.h"
#include "esp_netif.h"

/* Brings up the SPI bus, the W5500 MAC/PHY, and an esp_netif for it (DHCP client by
 * default; set ETH_USE_STATIC_IP in config.yaml for a fixed address). Safe to call
 * alongside Wi-Fi (web_ui) - they use different peripherals.
 * Returns the netif so callers can query its IP, or NULL on failure (the device keeps
 * running without Ethernet / SCADA in that case). */
esp_netif_t *eth_w5500_start(void);

bool eth_w5500_link_is_up(void);
