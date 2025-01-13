# Project-wide settings and configurations
set(PROJECT_NAME "PicoAPRS" CACHE STRING "Project name")
set(PROJECT_VERSION "1.0.0" CACHE STRING "Project version")

# MCU Target validation
set(SUPPORTED_MCUS
    STM32L432KC
    #Add more supported MCUs here
)

#Check that target was provided
if(NOT TARGET_MCU)
    message(FATAL_ERROR 
        "\nTARGET_MCU not defined! \
        \n\nTo configure the build with a target MCU, use a command like this \
        \n\n  cmake -B build/Debug -DTARGET_MCU=STM32L432KC --preset Debug    (STM32L432KC target) \
        \n  cmake --build build/L432KC         # Build the project \
        \n\nSupported MCUs: ${SUPPORTED_MCUS}"
        )
endif()

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

# Project-specific features that aren't in presets
option(ENABLE_DATA_LOGGING "Enable data logging features" ON)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DFEATURE_DEBUG)
endif()

if(ENABLE_DATA_LOGGING)
    add_definitions(-DFEATURE_DATA_LOGGING)
endif()