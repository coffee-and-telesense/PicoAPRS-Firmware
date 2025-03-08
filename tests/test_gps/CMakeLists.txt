cmake_minimum_required(VERSION 3.22)

# Include project configuration
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/test_config.cmake)

# Set project name first
set(PROJECT_NAME "test_gps")

# Set the target MCU - this is the only line users need to change
set(TARGET_MCU "STM32L432KC")
add_compile_definitions(TARGET_MCU=${TARGET_MCU})

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

## Set options for the gps lib
set(BUILD_BLOCKING_GPS_LIB ON CACHE BOOL "Build the blocking GPS library" FORCE)
set(BUILD_NON_BLOCKING_GPS_LIB OFF CACHE BOOL "Build the non-blocking GPS library" FORCE)

# Enable compile command for indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Define project
project(${PROJECT_NAME}
    DESCRIPTION "GPS Test Application"
)


message("Build type: ${CMAKE_BUILD_TYPE}")
message("Target MCU: ${TARGET_MCU}")

# Create executable
add_executable(${PROJECT_NAME})

# Add test sources
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/test_gps.c
)

# Add include paths
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Config/Inc
)

# Add STM32CubeMX generated sources
add_subdirectory(
    ${CUBEMX_PATH}/                       # Source directory
    ${CMAKE_BINARY_DIR}/stm32cubemx       # Binary directory
)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../libs
    ${CMAKE_BINARY_DIR}/libs
)

target_link_libraries(${PROJECT_NAME}
    stm32cubemx
    gps_driver_blocking
    common
)
