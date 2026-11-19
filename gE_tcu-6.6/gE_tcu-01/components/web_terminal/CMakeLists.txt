idf_component_register(SRCS "web_terminal.c" "terminal_page.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_http_server freertos log)
