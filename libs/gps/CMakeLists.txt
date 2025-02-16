# Library: ublox_gps_driver
cmake_minimum_required(VERSION 3.22)
project(ublox_gps_driver)

add_library(ublox_gps_driver STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/u-blox_gnss_MAX-M10S.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/u-blox_packet_handler.c
)

target_compile_definitions(ublox_gps_driver PUBLIC
    STM32L432xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(ublox_gps_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Config/Inc
)

# Link against the stm32cubemx INTERFACE Library
target_link_libraries(ublox_gps_driver PUBLIC
    common
    stm32cubemx
)
