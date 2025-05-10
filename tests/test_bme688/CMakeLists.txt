# Create an executable object type
add_executable(test_bme688 test_bme688.c)

target_sources(test_bme688 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test_bme688.c
)

# Add project symbols (macros)
target_compile_definitions(test_bme688 PRIVATE
    BME68X_DO_NOT_USE_FPU
)

target_link_libraries(test_bme688 PRIVATE 
    stm32cubemx
    common
    bme68x
)
