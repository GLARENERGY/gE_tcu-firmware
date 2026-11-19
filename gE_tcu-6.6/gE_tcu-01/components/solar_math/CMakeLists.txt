# components/solar_math/CMakeLists.txt
idf_component_register(SRCS "solar_math.c"
                       INCLUDE_DIRS "include"
                       REQUIRES soltrack nvs_flash board_hal)
