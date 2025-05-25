/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PGOOD_Pin GPIO_PIN_0
#define PGOOD_GPIO_Port GPIOA
#define PGOOD_EXTI_IRQn EXTI0_1_IRQn
#define SNSR_EN_Pin GPIO_PIN_1
#define SNSR_EN_GPIO_Port GPIOA
#define GPIO1_Pin GPIO_PIN_2
#define GPIO1_GPIO_Port GPIOA
#define GPIO0_Pin GPIO_PIN_3
#define GPIO0_GPIO_Port GPIOA
#define GPS_RTC_PWR_EN_Pin GPIO_PIN_0
#define GPS_RTC_PWR_EN_GPIO_Port GPIOB
#define GPS_PWR_EN_Pin GPIO_PIN_1
#define GPS_PWR_EN_GPIO_Port GPIOB
#define LLD_Pin GPIO_PIN_15
#define LLD_GPIO_Port GPIOA
#define RF_IRQ_Pin GPIO_PIN_3
#define RF_IRQ_GPIO_Port GPIOB
#define RF_IRQ_EXTI_IRQn EXTI2_3_IRQn
#define RF_EN_Pin GPIO_PIN_4
#define RF_EN_GPIO_Port GPIOB
#define CHRGR_EN_Pin GPIO_PIN_5
#define CHRGR_EN_GPIO_Port GPIOB
#define GPIO3_Pin GPIO_PIN_6
#define GPIO3_GPIO_Port GPIOB
#define GPIO2_Pin GPIO_PIN_7
#define GPIO2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
