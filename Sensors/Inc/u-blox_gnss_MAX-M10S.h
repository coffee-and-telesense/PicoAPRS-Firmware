#pragma once

#ifndef TARGET_MCU
    #error "MCU_TARGET must be defined"
#endif

//@TODO: Add more MCU_TARGETs
// For example MCU_TARGET == F091RC
#if TARGET_MCU == STM32L432KC
    #include "stm32l4xx_hal.h"
#endif

#include "u-blox_Class_and_ID.h"



