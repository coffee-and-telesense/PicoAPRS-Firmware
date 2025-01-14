cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME test_gps)

# Hardcode both the target name and path
set(TARGET_MCU "L432KC")
set(TARGET_PATH ${CMAKE_SOURCE_DIR}/../../targets/stm32L4xx/L432KC)

# Define the TARGET_MCU
add_compile_definitions(TARGET_MCU=STM32L432KC)

# Include the target's toolchain file
include(${TARGET_PATH}/cmake/gcc-arm-none-eabi.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable Language Support
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})

# Create test executable
add_executable(${CMAKE_PROJECT_NAME})

# Add test sources
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    test_gps.c
    ${CMAKE_SOURCE_DIR}/../../Src/Sensors/Src/u-blox_gnss_MAX-M10S.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/../../Src/Sensors/Inc
    ${TARGET_PATH}/Inc
)

# Add the target's CubeMX library - specify both source and binary dirs
add_subdirectory(
    ${TARGET_PATH}/cmake/stm32cubemx      # Source directory
    ${CMAKE_BINARY_DIR}/stm32cubemx       # Binary directory
)

target_link_libraries(${CMAKE_PROJECT_NAME} 
    stm32cubemx
)