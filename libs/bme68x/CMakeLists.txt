# BME68x Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(bme68x)

# Create the GPS library
add_library(bme68x STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/driver/max_m10s.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/protocols/ubx/ubx.c
)

# Define include directories
target_include_directories(bme68x PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols/ubx
)

# Add the NON_BLOCKING definition if the option is enabled
if(GPS_NON_BLOCKING)
    target_compile_definitions(bme68x PUBLIC NON_BLOCKING)
    message(STATUS "Building BME68x library in non-blocking mode")
else()
    message(STATUS "Building BME68x library in blocking mode")
endif()

# Link with required dependencies
target_link_libraries(bme68x PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added BME68x library")