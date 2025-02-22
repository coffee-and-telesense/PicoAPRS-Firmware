cmake_minimum_required(VERSION 3.22)

# Set project name first
set(PROJECT_NAME "test_gps_nb")

# Include project configuration
include(${CMAKE_SOURCE_DIR}/../../cmake/test_config.cmake)

# Set the target MCU - this is the only line users need to change
set(TARGET_MCU "STM32L432KC")

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Configure target-specific paths and settings
configure_target(${TARGET_MCU})

# Set linker script path for toolchain
set(LINKER_SCRIPT_PATH ${LINKER_SCRIPT} CACHE STRING "Path to linker script")

# Include the target's toolchain file
include(${TARGET_PATH}/cmake/gcc-arm-none-eabi.cmake)

# Enable compile command for indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Define project
project(${PROJECT_NAME}
    DESCRIPTION "Test project for ${TARGET_MCU}"
    LANGUAGES C CXX ASM
    VERSION 1.0
)

message("Build type: ${CMAKE_BUILD_TYPE}")
message("Target MCU: ${TARGET_MCU}")

# Create executable
add_executable(${PROJECT_NAME})

# Add test sources
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/test_gps_nb.c
)

# Add include paths
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/../../Config/Inc
)

# Add STM32CubeMX generated sources
add_subdirectory(
    ${CUBEMX_PATH}/                       # Source directory
    ${CMAKE_BINARY_DIR}/stm32cubemx       # Binary directory
)

add_subdirectory(
    ${CMAKE_SOURCE_DIR}/../../libs
    ${CMAKE_BINARY_DIR}/libs
)

target_link_libraries(${PROJECT_NAME}
    stm32cubemx
    gps_driver_nb
    common
)
