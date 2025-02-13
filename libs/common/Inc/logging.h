/*******************************************************************************
 * @file: logging.h
 * @brief: Simple debug printing utility for STM32 projects
 * 
 * @note: Requires UART to be initialized before use
 * 
 * @todo: This utility is very basic and could be expanded to include more features
 ******************************************************************************/

#pragma once

#include "main.h"
#include <string.h>
#include <stdio.h> // Required for snprintf()

#ifdef DEBUG
    #define DEBUG_UART huart2
    #define debug_print(fmt, ...)                                                                                    \
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
    #define debug_print(msg)
#endif
