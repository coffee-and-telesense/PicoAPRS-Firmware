# AX25 Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(AX25)

# Create the ax25 library
add_library(ax25 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src
)

# Define include directories
target_include_directories(ax25 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Add the NON_BLOCKING definition if the option is enabled
if(AX25_NON_BLOCKING)
    target_compile_definitions(ax25 PUBLIC NON_BLOCKING)
    message(STATUS "Building ax25 library in non-blocking mode")
else()
    message(STATUS "Building ax25 library in blocking mode")
endif()

# Link with required dependencies
target_link_libraries(ax25 PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added ax25 library")