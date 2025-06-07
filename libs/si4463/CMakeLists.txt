# SI4463 Library CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(si4463)

# Create the SI4463 library
add_library(si4463 STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Src/si_trx.c
)

# Define include directories
target_include_directories(si4463 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Link with required dependencies
target_link_libraries(si4463 PUBLIC
    stm32cubemx
    # Add dependencies here
)

message(STATUS "Added SI4463 library")