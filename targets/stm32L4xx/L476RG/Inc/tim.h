/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN Private defines */
extern volatile uint8_t delay_complete;

/* USER CODE END Private defines */

void MX_TIM2_Init(void);

/* USER CODE BEGIN Prototypes */

/**
 * @brief Example implementation for a microsecond delay callback
 *
 * Used to set the delay_us function pointer on the bme68x_dev device struct.
 * This callback is used throughout the Bosch library whenever a microsecond
 * delay is necessary for proper device functioning.
 *
 * @param[in] period_us Duration of the delay in microseconds
 * @param[in] intf_ptr Pointer to the interface descriptor
 * 
 * @note: intf_ptr is not used by this function, but needed to satisfy the
 * callback function signature
 */
void delay_us_timer(uint32_t us, void *intf_ptr);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

