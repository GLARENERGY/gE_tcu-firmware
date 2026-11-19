idf_component_register(SRCS "web_ui.c" "web_assets_gen.c"
                       INCLUDE_DIRS "include" "."
                       REQUIRES anemometer tracker_core solar_math board_hal esp32_drivers nvs_flash
                                esp_wifi esp_event esp_netif esp_http_server mdns
                                esp_system esp_hw_support spiffs esp_timer web_terminal)
