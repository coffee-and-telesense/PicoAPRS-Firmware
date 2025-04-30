# BME68x Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(bme68x_test)

# Create the BME68X library
add_library(bme68x_test STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bme68x_driver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/bme68x.c
)

# Define include directories
target_include_directories(bme68x_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols/ubx
)

# Link with required dependencies
target_link_libraries(bme68x_test PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added bme68x_test library")