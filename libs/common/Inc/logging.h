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

#ifdef DEBUG
    #define DEBUG_UART huart2
    #define debug_print(msg) do { \
        HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)(msg), strlen(msg), 100); \
    } while(0)
#else
    #define debug_print(msg)
#endif