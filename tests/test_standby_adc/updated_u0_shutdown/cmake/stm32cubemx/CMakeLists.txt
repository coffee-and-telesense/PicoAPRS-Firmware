cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32U083xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32U0xx_HAL_Driver/Inc
    ../../Drivers/STM32U0xx_HAL_Driver/Inc/Legacy
    ../../Drivers/CMSIS/Device/ST/STM32U0xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/gpio.c
    ../../Core/Src/adc.c
    ../../Core/Src/rtc.c
    ../../Core/Src/usart.c
    ../../Core/Src/stm32u0xx_it.c
    ../../Core/Src/stm32u0xx_hal_msp.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_adc.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_adc_ex.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_ll_adc.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_rcc.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_rcc_ex.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_flash.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_flash_ex.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_gpio.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_dma.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_dma_ex.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_pwr.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_pwr_ex.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_cortex.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_exti.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_rtc.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_rtc_ex.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_uart.c
    ../../Drivers/STM32U0xx_HAL_Driver/Src/stm32u0xx_hal_uart_ex.c
    ../../Core/Src/system_stm32u0xx.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32u083xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


