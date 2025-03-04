cmake_minimum_required(VERSION 3.22)

# Define project metadata
project(PicoAPRS-Firmware
    DESCRIPTION "PicoAPRS Firmware"
    LANGUAGES C CXX ASM
    VERSION 0.1.0
)

# Set basic compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Print basic configuration information
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
if(DEFINED TARGET_MCU)
    message(STATUS "Target MCU: ${TARGET_MCU}")
endif()

# Always add the libraries directory
#add_subdirectory(libs)

# Check for specific test selections
if(DEFINED BUILD_TEST_GPS AND BUILD_TEST_GPS)
    message(STATUS "Building GPS test application")
    add_subdirectory(tests/test_gps)
elseif(DEFINED BUILD_TEST_GPS_NB AND BUILD_TEST_GPS_NB)
    message(STATUS "Building non-blocking GPS test application")
    add_subdirectory(tests/test_gps_nb)
elseif(DEFINED BUILD_TEST_BME688 AND BUILD_TEST_BME688)
    message(STATUS "Building BME688 test application")
    add_subdirectory(tests/test_bme688)
else()
    message(STATUS "No specific test selected. Use a preset to select a test application.")
endif()
