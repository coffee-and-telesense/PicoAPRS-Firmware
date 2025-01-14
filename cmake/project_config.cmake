# Project-wide settings and configurations
set(PROJECT_NAME "PicoAPRS" CACHE STRING "Project name")
set(PROJECT_VERSION "1.0.0" CACHE STRING "Project version")

# MCU Target validation
set(SUPPORTED_MCUS
    STM32L432KC
    #Add more supported MCUs here
)

# Check if target was provide else, use default configuration
if (DEFINED TARGET_MCU)
    set(CMAKE_TARGET_MCU ${TARGET_MCU} CACHE STRING "Target MCU")
else()
    set(CMAKE_TARGET_MCU "STM32L432KC" CACHE STRING "Target MCU")
    message(WARNING 
        "\nTARGET_MCU not defined, defaulting to STM32L432KC \
        \nTo specify a different target, use: \
        \n  cmake -B build/Debug -DTARGET_MCU=<target> --preset Debug"
    )
endif()

add_compile_definitions(TARGET_MCU=${CMAKE_TARGET_MCU})

# Validate if the provided TARGET_MCU is supported
list(FIND SUPPORTED_MCUS ${TARGET_MCU} MCU_INDEX)
if(MCU_INDEX EQUAL -1)
    message(FATAL_ERROR
        "\nUnsupported TARGET_MCU: ${TARGET_MCU} \
        \n\nPlease use one of these supported MCUs: \
        \n  ${SUPPORTED_MCUS} \
        \n\nExample command: \
        \n  cmake -B build/Debug -DTARGET_MCU=STM32L432KC --preset Debug"
    )
endif()

# UBX Protocol Configuration
option(ENABLE_UBX_PROTOCOL "Enable UBX protocol support" ON)
option(UBX_USE_I2C "Use I2C for UBX communication" ON)
option(UBX_USE_UART "Use UART for UBX communication" OFF)

# Validate UBX configuration
if(ENABLE_UBX_PROTOCOL)
    if(NOT UBX_USE_I2C AND NOT UBX_USE_UART)
        message(FATAL_ERROR "UBX protocol enabled but no communication interface selected (I2C or UART)")
    endif()
    
    # Add UBX-related definitions
    add_definitions(-DFEATURE_UBX_PROTOCOL)
    
    if(UBX_USE_I2C)
        add_definitions(-DUBX_USE_I2C)
    endif()
    
    if(UBX_USE_UART)
        add_definitions(-DUBX_USE_UART)
    endif()
endif()

# Project-specific features, for example:
# option(ENABLE_DATA_LOGGING "Enable data logging features" ON)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DFEATURE_DEBUG)
endif()

if(ENABLE_DATA_LOGGING)
    add_definitions(-DFEATURE_DATA_LOGGING)
endif()