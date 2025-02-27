cmake_minimum_required(VERSION 3.22)
project(bme68x)

add_library(bme68x STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bme68x.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bme68x_driver.c
)

target_compile_definitions(bme68x PUBLIC
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(bme68x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Config/Inc
)

target_link_libraries(bme68x PUBLIC
    stm32cubemx
    common  # Inherit common's includes
)
