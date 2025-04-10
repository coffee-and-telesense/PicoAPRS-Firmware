# GPS Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(gps)

# Create the GPS library
add_library(gps STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/driver/max_m10s.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/protocols/ubx/ubx.c
)

# Define include directories
target_include_directories(gps PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/driver
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc/protocols/ubx
)

# Add the NON_BLOCKING definition if the option is enabled
if(GPS_NON_BLOCKING)
    target_compile_definitions(gps PUBLIC NON_BLOCKING)
    message(STATUS "Building GPS library in non-blocking mode")
else()
    message(STATUS "Building GPS library in blocking mode")
endif()

# Link with required dependencies
target_link_libraries(gps PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added GPS library")