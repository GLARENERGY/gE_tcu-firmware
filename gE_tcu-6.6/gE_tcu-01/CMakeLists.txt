cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "lib/pcf2131"
    "src"
)

# Generate AppConfig.h from config.yaml (edit pins in config.yaml, not in the header).
# If generation fails (e.g. PyYAML missing) the existing header is kept.
set(CONFIG_YAML   "${CMAKE_CURRENT_SOURCE_DIR}/config.yaml")
set(CONFIG_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/components/board_hal/include/AppConfig.h")
set(DEFINEPARSE_PY "${CMAKE_CURRENT_SOURCE_DIR}/scripts/defineparse.py")
if(NOT PYTHON)
    find_program(PYTHON NAMES python3 python)
endif()
execute_process(COMMAND "${PYTHON}" "${DEFINEPARSE_PY}" "${CONFIG_YAML}" "${CONFIG_HEADER}"
                RESULT_VARIABLE gen_result OUTPUT_VARIABLE gen_out ERROR_VARIABLE gen_err)
if(NOT gen_result EQUAL 0)
    message(WARNING "AppConfig.h NOT regenerated (${gen_err}). Using the existing header.")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CONFIG_YAML}")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(gE_tcu_01)
