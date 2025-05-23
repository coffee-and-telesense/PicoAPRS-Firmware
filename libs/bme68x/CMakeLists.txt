# BME68x Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(bme68x)

# Create the BME68X library
add_library(bme68x STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bme68x_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bme68x.c
)

target_compile_definitions(bme68x PUBLIC
    BME68X_DO_NOT_USE_FPU
    $<$<CONFIG:Debug>:DEBUG>
)

# Define include directories
target_include_directories(bme68x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bme68x_defs.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bme68x_driver.h
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/bme68x
)

# Link with required dependencies
target_link_libraries(bme68x PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added BME68x library")