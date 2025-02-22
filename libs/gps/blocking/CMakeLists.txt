# Library: ublox_gps_driver
cmake_minimum_required(VERSION 3.22)
project(gps_driver_blocking)

add_library(gps_driver_blocking STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/driver/max_m10s.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/protocols/ubx/ubx_packet_handler.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/protocols/ubx/ubx_protocol.c
)

target_compile_definitions(ublox_gps_driver PUBLIC
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(ublox_gps_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/core
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols/ubx
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Config/Inc
)

# Link against the stm32cubemx INTERFACE Library
target_link_libraries(ublox_gps_driver PUBLIC
    common
    stm32cubemx
)
