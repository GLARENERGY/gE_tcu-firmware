# components/soltrack/CMakeLists.txt
# This is a wrapper for the external SolTrack library to keep it untouched.

set(SOLTRACK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../extern/soltrack")

idf_component_register(SRCS "${SOLTRACK_DIR}/SolTrack.c" "${SOLTRACK_DIR}/SolTrack_RiseSet.c"
                       INCLUDE_DIRS "${SOLTRACK_DIR}")
