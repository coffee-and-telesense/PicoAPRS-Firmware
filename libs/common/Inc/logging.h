/*******************************************************************************
 * @file: logging.h
 * @brief: Simple debug printing utility for STM32 projects
 * 
 * @note: Requires UART to be initialized before use
 * 
 * @todo: This utility is very basic and could be expanded to include more features
 ******************************************************************************/

#pragma once

#include "stm32l4xx_hal.h"
#include "usart.h"
extern UART_HandleTypeDef huart2;

// The following macro may be used when inlining the code is preferable for
// performance reasons, and the additional code size is not a concern.
// A debug_print function is available in logger.c to provide equivalent
// functionality with a reduced code footprint, but likely slower performance.
#ifdef DEBUG

    #include <string.h>
    #include <stdio.h> // Required for snprintf()
    #include <stdarg.h> // Required for va_list handling

    void debug_print(const char *fmt, ...);

    #define DEBUG_UART huart2
    #define debug_print_inline(fmt, ...)                                                                             \
    do                                                                                                               \
    {                                                                                                                \
        char buffer[128]; /* Adjust size as needed */                                                                \
        int result = snprintf(buffer, sizeof(buffer), fmt, ##__VA_ARGS__);                                           \
        if (result < 0)                                                                                              \
        {                                                                                                            \
            /* Handle error */                                                                                       \
            HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)"snprintf error\n", 15, 100);                                  \
        }                                                                                                            \
        else                                                                                                         \
        {                                                                                                            \
            /* Cast to size_t for the comparison to sizeof result below */                                           \
            size_t len = (size_t)result;                                                                             \
            buffer[sizeof(buffer) - 1] = '\0'; /* Explicit null termination */                                       \
            HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)buffer, len > sizeof(buffer) ? sizeof(buffer) - 1 : len, 100); \
        }                                                                                                            \
    } while (0)
#else
    #define debug_print_inline(msg)
#endif
