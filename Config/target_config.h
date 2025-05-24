#pragma once

/*******************************************************************************
 * @brief: Target MCU Configuration
 * @purpose: This file is used to define target MCU definitions and simplifies the
 *         configuration of the project. Instead of conditionally including specific HALs
 *         throughout the source code, this file handles these definitions in one place. Hence, 
 *         the source code can be written in a more generic way and portable way across stm32 MCUs.
 * 
 * Supported MCUs:
 * - U083RC, U073KCU
 * @todo: Add more MCUs
 ******************************************************************************/

#ifndef TARGET_MCU
    #error "TARGET_MCU must be defined"
#endif

/* STM32L4 Series Configuration */
#if (TARGET_MCU == U083) || (TARGET_MCU == U073) // @TODO: Add more MCUs as needed
    //U0 Family HAL
    #include "stm32U0xx_hal.h"

    #if defined(U083)
        //@todo: Add more granularity to the U083 as needed
        
    #endif

    #if defined(U073)
        //@todo: Add more granularity to the U073 as needed
        
    #endif


#endif

/*Add more families or chips as needed*/