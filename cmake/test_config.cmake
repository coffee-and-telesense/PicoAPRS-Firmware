# /cmake/test_config.cmake
#
# This file provides configuration and utility functions for building individual
# test projects targeting specific STM32 microcontrollers. It handles:
#
# - Target validation and configuration
# - Path resolution for target-specific files (cubemx, linker scripts)
#
# Usage:
# In your test project's CMakeLists.txt:
# 1. include(${CMAKE_SOURCE_DIR}/../../cmake/test_config.cmake)
# 2. set(TARGET_MCU "STM32L432KC")  # Select your target
# 3. configure_target(${TARGET_MCU}) # Configure paths and toolchain
#
# This system makes it easy to create new test projects targeting different
# STM32 MCUs while reusing common build infrastructure.

set(SUPPORTED_TARGETS
    "STM32L432KC"
    "STM32L476RG"
    "STM32U083RC"
    # TODO: Add more targets as needed
)

# Target-specific configurations
set(STM32L432KC_PATH "${CMAKE_SOURCE_DIR}/../../targets/stm32L4xx/L432KC")
set(STM32L432KC_CUBEMX_PATH "${STM32L432KC_PATH}/cmake/stm32cubemx")

set(STM32L476RG_PATH "${CMAKE_SOURCE_DIR}/../../targets/stm32L4xx/L476RG")
set(STM32L476RG_CUBEMX_PATH "${STM32L476RG_PATH}/cmake/stm32cubemx")

set(STM32U083RC_PATH "${CMAKE_SOURCE_DIR}/../../targets/stm32U0xx/U083RC")
set(STM32U083RC_CUBEMX_PATH "${STM32U083RC_PATH}/cmake/stm32cubemx")
# TODO: Add more target specific paths as needed

# Function to validate target selection
function(validate_target TARGET)
    list(FIND SUPPORTED_TARGETS ${TARGET} INDEX)
    if(INDEX EQUAL -1)
        message(FATAL_ERROR "[test_config.cmake] Invalid target '${TARGET}'. Supported targets are: ${SUPPORTED_TARGETS}")
    endif()
endfunction()

# Function to find linker scripter
function(find_linker_script TARGET_PATH RESULT)
    file(GLOB LINKER_SCRIPTS "${TARGET_PATH}/*.ld")
    list(LENGTH LINKER_SCRIPTS SCRIPT_COUNT)
    
    if(SCRIPT_COUNT EQUAL 0)
        message(FATAL_ERROR "[test_config.cmake] No linker script found in ${TARGET_PATH}")
    elseif(SCRIPT_COUNT GREATER 1)
        message(FATAL_ERRORR "[test_config.cmake] Multiple linker scripts found in ${TARGET_PATH}, using first one")
    endif()
    
    list(GET LINKER_SCRIPTS 0 SCRIPT_PATH)
    set(${RESULT} ${SCRIPT_PATH} PARENT_SCOPE)
    message(STATUS "[test_config.cmake] Linker script: ${SCRIPT_PATH}")
endfunction()


# Function to configure target-specific settings
function(configure_target TARGET)
    message(STATUS "[test_config.cmake] Configuring target paths from cmake/test_config.cmake")
    validate_target(${TARGET})
   
    # Store path in local variable first
    set(LOCAL_TARGET_PATH ${${TARGET}_PATH})
    set(LOCAL_CUBEMX_PATH ${${TARGET}_CUBEMX_PATH})
    
    # Set for parent scope
    set(TARGET_PATH ${LOCAL_TARGET_PATH} PARENT_SCOPE)
    set(CUBEMX_PATH ${LOCAL_CUBEMX_PATH} PARENT_SCOPE)
   
    message(STATUS "[test_config.cmake] Linker script lookup starting...")
    find_linker_script(${LOCAL_TARGET_PATH} LINKER_SCRIPT)
    message(STATUS "[test_config.cmake] Found linker script: ${LINKER_SCRIPT}")
    set(LINKER_SCRIPT_PATH ${LINKER_SCRIPT} CACHE STRING "Path to linker script" FORCE)
   
    add_compile_definitions(TARGET_MCU=${TARGET})

    message(STATUS "-- [test_config.cmake] Target path: ${LOCAL_TARGET_PATH}")
    message(STATUS "-- [test_config.cmake] CubeMX path: ${LOCAL_CUBEMX_PATH}")
    message(STATUS "-- [test_config.cmake] Done configuring target paths")
endfunction()
