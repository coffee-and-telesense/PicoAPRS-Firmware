cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME "test_i2c")

# Include project configuration
include(${CMAKE_SOURCE_DIR}/../../cmake/test_config.cmake)

# Set the target MCU - this is the only line users need to change
set(TARGET_MCU "STM32U083RC")

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

## Set options for the gps lib
set(BUILD_BLOCKING_GPS_LIB ON CACHE BOOL "Build the blocking GPS library" FORCE)
set(BUILD_NON_BLOCKING_GPS_LIB OFF CACHE BOOL "Build the non-blocking GPS library" FORCE)

# Configure target-specific paths and settings
configure_target(${TARGET_MCU})

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set linker script path for toolchain
set(LINKER_SCRIPT_PATH ${LINKER_SCRIPT} CACHE STRING "Path to linker script")

# Include the target's toolchain file
include(${TARGET_PATH}/cmake/gcc-arm-none-eabi.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)


project(${PROJECT_NAME} VERSION 1.0
                  DESCRIPTION "Test I2C Integration"
                  LANGUAGES C CXX ASM
)  

message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${PROJECT_NAME})

# Add test sources
target_sources(${PROJECT_NAME} PRIVATE
    # Add user sources here
    # Example: ${CMAKE_SOURCE_DIR}/test_gps.c
    ${CMAKE_SOURCE_DIR}/test_i2c.c
)

# Add include paths
target_include_directories(${PROJECT_NAME} PRIVATE
    # Add user defined include paths here
    # Example: ${CMAKE_SOURCE_DIR}/../../Src/Sensors/Inc
    ${CMAKE_SOURCE_DIR}/../../Config/Inc
    ${CMAKE_SOURCE_DIR}/../../gps/blocking/driver
)



# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols for local scope to target executable
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

target_link_libraries(${CMAKE_PROJECT_NAME} 
    stm32cubemx
    gps_driver_blocking    
    common
)

# --------------------------------------------------
# Enable float printing with newlib-nano:
target_link_options(${PROJECT_NAME} PRIVATE
    "-u_printf_float"
)
# --------------------------------------------------