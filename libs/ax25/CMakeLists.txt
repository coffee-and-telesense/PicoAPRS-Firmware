# AX25 Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(ax25)

# Create the ax25 library
add_library(ax25 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/ax25.c
)

# Define include directories
target_include_directories(ax25 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Link with required dependencies
target_link_libraries(ax25 PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added ax25 library")