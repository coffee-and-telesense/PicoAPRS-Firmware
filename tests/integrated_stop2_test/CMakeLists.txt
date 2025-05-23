add_executable(integrated_stop2_test integrated_stop2_test.c)

target_sources(integrated_stop2_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/integrated_stop2_test.c
)

target_link_libraries(integrated_stop2_test PRIVATE 
    stm32cubemx
    gps
    bme68x
    aprs
    ax25
    common
)