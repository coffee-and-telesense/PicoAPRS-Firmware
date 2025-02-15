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
#include "logging.h"
#include "bme68x_driver.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* Defines ------------------------------------------------------------*/

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint8_t Read_Register(uint8_t reg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  debug_print_inline("Printing with macro: %d\r\n", 5);
  debug_print("Printing with function: %d\r\n", 10);

  if (HAL_I2C_IsDeviceReady(&hi2c1, BME68X_ADDR, 3, HAL_MAX_DELAY) == HAL_OK) {
    debug_print("Sensor is ready\r\n");
  } else {
    debug_print("Sensor not responding\r\n");
  }
  // Create bme instance / struct
  // Initialize or begin
  // Check status
  // bme.setTPH (set temp, pressure, humidity config)
  // bme.setHeaterProf(int, int) (set heater profile)
  // Initialize bme data struct (here or in loop??)
  // bme.setOpMode(forced_mode_enum) (here or in loop??)

  uint8_t sensor_id = Read_Register(0xD0);
  debug_print("Received sensor ID: 0x%X\r\n", sensor_id);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // (maybe, if not done above): bme data and setOpMode
    // (maybe) bme.getMeasDur()
    // bme.fetchData() retrieve data from sensor
    // bme.getData() get data
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(1000);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
uint8_t Read_Register(uint8_t reg)
{
  uint8_t data = 0;
  HAL_I2C_Master_Transmit(&hi2c1, BME68X_ADDR, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, BME68X_ADDR, &data, 1, HAL_MAX_DELAY);
  return data;
}

/* USER CODE END 4 */
