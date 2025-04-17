# APRS Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(aprs)

# Create the GPS library
add_library(aprs STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src
)

# Define include directories
target_include_directories(aprs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Link with required dependencies
target_link_libraries(aprs PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added APRS library")