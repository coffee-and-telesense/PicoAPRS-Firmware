add_executable(test_bme688 test_bme688.c)

target_sources(test_bme688 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test_bme688.c
)

target_link_libraries(test_bme688 PRIVATE 
    stm32cubemx
    bme68x
    common
)