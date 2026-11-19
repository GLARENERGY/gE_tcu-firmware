# components/tracker_core/CMakeLists.txt
idf_component_register(SRCS "tracker_core.c"
                       INCLUDE_DIRS "include"
                       REQUIRES board_hal solar_math anemometer espressif__cjson)
