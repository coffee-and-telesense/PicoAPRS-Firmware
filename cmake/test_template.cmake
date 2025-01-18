cmake_minimum_required(VERSION 3.22)

# FIXME: Set project name first
set(PROJECT_NAME "test_gps")

# Include project configuration
include(${CMAKE_SOURCE_DIR}/../../cmake/test_config.cmake)

# FIXME: Set the target MCU, must follow specific naming convention as defined in test_config.cmake
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

# FIXME: Add test sources must have one reference to main() function
target_sources(${PROJECT_NAME} PRIVATE
    # Example ${CMAKE_SOURCE_DIR}/test_gps.c
)


target_include_directories(${PROJECT_NAME} PRIVATE
    #Example ${CMAKE_SOURCE_DIR}/../../Src/Sensors/Inc
    # Add include paths
)

# Add STM32CubeMX generated sources
# add_subdirectory(${CUBEMX_PATH} ${CMAKE_BINARY_DIR}/stm32cubemx)
add_subdirectory(
    ${CUBEMX_PATH}/                       # Source directory
    ${CMAKE_BINARY_DIR}/stm32cubemx       # Binary directory
)

target_link_libraries(${PROJECT_NAME} 
    stm32cubemx
    # Add libraries
)