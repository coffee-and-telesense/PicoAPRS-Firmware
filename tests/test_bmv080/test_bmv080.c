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
#include "bmv080_example.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint8_t Read_Register(uint8_t reg);

/* Print function */
int bmv080_uart_print(const char *fmt, ...);

void enable_external_interrupt(bool enable)
{
  if (enable)
  { /* Enabel external interrupt */
    // HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  }
  else
  { /* Disable external interrupt */
    // HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  }
}

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

  debug_print("BMV080 Test\r\n");

  // TODO: Check address - the define BMV080_I2C_ADDRESS is 0x54, from the Bosch example code,
  // but the default address is 0x57. The default of 0x57 works with a left shift, which seem
  // fine, though it's unclear why the example code defaults to 0x54.
  // Should but the working value into a define with the shift.
  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x57 << 1, 3, HAL_MAX_DELAY) == HAL_OK)
  {
    debug_print("Yes sensor is ready\r\n");
  }
  else
  {
    debug_print("Sensor not responding\r\n");
  }

  HAL_Delay(1000);

  // Got the following output with the call to bmv080 below:
  // BMV080 sensor driver version: 24.0.0.16b8c68a693.0
  // Opening BMV080 sensor unit failed with BMV080 status 106
  // So next TODO: Look into status 106 failure for bmv080_open

  /* Sensor API execution */
  bmv080_status_code_t bmv080_final_status = bmv080(
      (bmv080_sercom_handle_t)&hi2c1,
      (const bmv080_callback_read_t)combridge_i2c_read_16bit,
      (const bmv080_callback_write_t)combridge_i2c_write_16bit,
      (const bmv080_callback_delay_t)combridge_delay,
      (const bmv080_callback_tick_t)HAL_GetTick,
      (const print_function_t)bmv080_uart_print,
      (const enable_ext_interrupt_function_t)enable_external_interrupt);

  if (bmv080_final_status != E_BMV080_OK)
  {
    debug_print("Executing the sensor APIs failed with bmv080 status %d\n", (int)bmv080_final_status);
  }

  /* Infinite loop */
  while (1)
  {

    HAL_Delay(1000);
    debug_print("BMV080 test app loop\r\n");
  }
}

int bmv080_uart_print(const char *fmt, ...)
{
  char buffer[128];
  int result;

  va_list args;
  va_start(args, fmt);
  result = vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  if (result < 0)
  {
    const char *err_msg = "[bmv080_print] vsnprintf error\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)err_msg, strlen(err_msg), 100);
    return result;
  }

  buffer[sizeof(buffer) - 1] = '\0';
  size_t len = (size_t)result;
  if (len > sizeof(buffer))
    len = sizeof(buffer) - 1;

  HAL_UART_Transmit(&huart2, (uint8_t *)buffer, len, 100);
  return result;
}
