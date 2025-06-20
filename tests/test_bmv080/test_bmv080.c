/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "logging.h"
#include "bmv080_driver.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t Read_Register(uint8_t reg);

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  debug_print("************BMV080 Test************\r\n");

  if (HAL_I2C_IsDeviceReady(&hi2c1, BMV080_I2C_ADDRESS << 1, 3, HAL_MAX_DELAY) == HAL_OK)
  {
    debug_print("Success: BMV080 sensor is ready\r\n");
  }
  else
  {
    debug_print("Error: BMV080 is not responding\r\n");
  }

  HAL_Delay(1000);

  // BMV080 sensor driver version: 24.0.0.16b8c68a693.0
  bmv080_sensor_t bmv080;
  bmv080_init(&bmv080, &hi2c1);
  bmv080_configure_duty_cycle(&bmv080, 20); // 20 second cycle
  if (bmv080.status != E_BMV080_OK)
    printf("Failed to configure duty cycle: %d\r\n", bmv080.status);
  bmv080_start_duty_cycle(&bmv080);

  while (1)
  {
    if (bmv080_poll(&bmv080) == E_BMV080_OK && bmv080.data_available)
    {
      // Floating point output
      bmv080_print_output(&bmv080.output);

      // Convert to fixed-point representation
      bmv080_fixed_t fixed = bmv080_to_fixed(&bmv080.output);

      // Print fixed-point values (example)
      debug_print("Fixed output: runtime=%u (0.01s), PM1=%u, PM2.5=%u, PM10=%u, flags=0x%02X\r\n",
                  fixed.runtime_in_0_01_sec,
                  fixed.pm1,
                  fixed.pm2_5,
                  fixed.pm10,
                  fixed.flags);

      bmv080.data_available = false;
      debug_print("\r\n");
    }
    HAL_Delay(100); // Polling period
  }
}
