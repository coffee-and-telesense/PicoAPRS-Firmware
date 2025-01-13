#pragma once

#ifndef MCU_TARGET
#error "MCU_TARGET must be defined"
#endif

//@TODO: Add more MCU_TARGETs
// For example MCU_TARGET == F091RC
#if MCU_TARGET == L432KC
#include "stm32l4xx_hal.h"
#endif

#include "u-blox_Class_and_ID.h"



