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
#include "usart.h"
#include "gpio.h"
// FIXME: Remove string library after troubleshooting
#include <string.h>

/* Includes ----------------------------------------------------------*/

/* Typedef -----------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/
#define BME688_ADDRESS 0x77 << 1 // HAL expects 8-bit address (shifted left)

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

/* Function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void I2C_Sensor_Init(void);
void UART_Print(char *msg);
uint8_t Read_Register(uint8_t reg);
void Write_Register(uint8_t reg, uint8_t value);

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

  I2C_Sensor_Init();

  uint8_t sensor_id = Read_Register(0xD0); // Example: Reading an ID register

  /* Infinite loop */
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    /* Insert delay 100 ms */
    HAL_Delay(2000);
    uint8_t sensor_id = Read_Register(0xD0); // Example: Reading an ID register
  }
}

void I2C_Sensor_Init()
{
  char buffer[50];
  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, (0x77 << 1), 10, 100);

  if (status == HAL_OK)
  {
    strcpy(buffer, "Device 0x77 is ready!\n");
  }
  else if (status == HAL_ERROR)
  {
    strcpy(buffer, "I2C Error!\n");
  }
  else if (status == HAL_BUSY)
  {
    strcpy(buffer, "I2C Bus is busy!\n");
  }
  else if (status == HAL_TIMEOUT)
  {
    strcpy(buffer, "I2C Timeout!\n");
  }
  // if (HAL_I2C_IsDeviceReady(&hi2c1, BME688_ADDRESS, 10, HAL_MAX_DELAY) == HAL_OK)
  // {
  //   strcpy(buffer, "Sensor is ready\n");
  // }
  // else
  // {
  //   strcpy(buffer, "Sensor not responding\n");
  // }
  UART_Print(buffer);
}

uint8_t Read_Register(uint8_t reg)
{
  uint8_t data = 0;
  HAL_I2C_Master_Transmit(&hi2c1, BME688_ADDRESS, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, BME688_ADDRESS, &data, 1, HAL_MAX_DELAY);
  return data;
}

void Write_Register(uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};
  HAL_I2C_Master_Transmit(&hi2c1, BME688_ADDRESS, data, 2, HAL_MAX_DELAY);
}

void UART_Print(char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}