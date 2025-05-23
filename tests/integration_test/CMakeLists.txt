add_executable(integration_test integration_test.c)

target_sources(integration_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/integration_test.c
)

target_link_libraries(integration_test PRIVATE 
    stm32cubemx
    gps
    bme68x
    aprs
    ax25
    common
)