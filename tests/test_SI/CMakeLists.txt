add_executable(test_SI test_SI.c)

target_sources(test_aprs PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test_SI.c
)

target_link_libraries(test_aprs PRIVATE 
    stm32cubemx
    aprs
    ax25
    gps
    common
)