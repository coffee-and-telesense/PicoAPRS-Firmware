cmake_minimum_required(VERSION 3.22)

if(DEFINED TARGET_MCU)
    message(STATUS "Building for target: ${TARGET_MCU}")

    if(${TARGET_MCU} STREQUAL "U083")
        set(U083 ON)
    elseif(${TARGET_MCU} STREQUAL "U073KCU6")
        set(U073KCU6 ON)
    endif()
endif()


if(U083)
    # Use the existing toolchain file from CubeMX
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/targets/stm32U0xx/U083/cmake/gcc-arm-none-eabi.cmake)

    # Disable CTest integration
    set(CMAKE_SKIP_TESTS TRUE)

    project(PicoAPRS-RTOS-Firmware C CXX ASM)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    ### This is a generator expression which will be evaluated during build
    ### system generation to produce project wide definitions
    add_compile_definitions($<$<CONFIG:Debug>:DEBUG>)

    ### gps feature selection, see libs\gps\Inc\driver\max_m10s.h for details
    #(NON-BLOCKING if ON or BLOCKING if OFF)
    option(GPS_NON_BLOCKING "Enable GPS Blocking/Non-Blocking Feature" ON)

    # Add library directories
    add_subdirectory(targets/stm32U0xx/U083/cmake/stm32cubemx)
    add_subdirectory(libs/gps)
    add_subdirectory(libs/common)
    add_subdirectory(libs/bme68x)
    add_subdirectory(libs/aprs)
    add_subdirectory(libs/ax25)
endif()

if(U073KCU6)

# Use the existing toolchain file from CubeMX
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/targets/stm32U0xx/U073KCU6/cmake/gcc-arm-none-eabi.cmake)

    # Disable CTest integration
    set(CMAKE_SKIP_TESTS TRUE)

    project(PicoAPRS-RTOS-Firmware C CXX ASM)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

    ### This is a generator expression which will be evaluated during build
    ### system generation to produce project wide definitions
    add_compile_definitions($<$<CONFIG:Debug>:DEBUG>)

    ### gps feature selection, see libs\gps\Inc\driver\max_m10s.h for details
    #(NON-BLOCKING if ON or BLOCKING if OFF)
    option(GPS_NON_BLOCKING "Enable GPS Blocking/Non-Blocking Feature" ON)

    # Add library directories
    add_subdirectory(targets/stm32U0xx/U073KCU6/cmake/stm32cubemx)
    add_subdirectory(libs/gps)
    #add_subdirectory(libs/common)
    add_subdirectory(libs/bme68x)
    add_subdirectory(libs/aprs)
    add_subdirectory(libs/ax25)

endif()

if(DEFINED LINKER_SCRIPT)
    message(STATUS "Using custom linker script: ${LINKER_SCRIPT}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT}")
endif()



# Add tests
#add_subdirectory(tests/gps_simple_test)
add_subdirectory(tests/integration_test)
add_subdirectory(tests/integrated_stop2_test)
add_subdirectory(tests/test_aprs)
#add_subdirectory(tests/test_bme688)

