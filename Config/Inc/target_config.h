#pragma once

/*******************************************************************************
 * @brief: Target MCU Configuration
 * @purpose: This file is used to define target MCU definitions and simplifies the
 *         configuration of the project. Instead of conditionally including specific HALs
 *         throughout the source code, this file handles these definitions in one place. Hence, 
 *         the source code can be written in a more generic way and portable way across stm32 MCUs.
 * 
 * Supported MCUs:
 * - L432KC
 * @todo: Add more MCUs
 ******************************************************************************/

#ifndef TARGET_MCU
    #error "TARGET_MCU must be defined"
#endif

/* STM32L4 Series Configuration */
#if (TARGET_MCU == STM32L432KC) || (TARGET_MCU == STM32L476RG) // @TODO: Add more MCUs as needed
    //L4 Family HAL
    #include "stm32l4xx_hal.h"

    /* L4 Series Defines*/
    #define MCU_FAMILY_L4

    #if defined(STM32L432KC)
        //@todo: Add more granularity to the L432KC as needed
    #endif


#endif

/*Add more families or chips as needed*/
