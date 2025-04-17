add_executable(integration_test test_integration.c)

target_sources(integration_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test_integration.c
)

target_link_libraries(integration_test PRIVATE 
    stm32cubemx
    gps
    bme688
    aprs
    ax25
    common
)