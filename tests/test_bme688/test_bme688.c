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
  bme68x_sensor_t bme;
  // Initialize / begin
  bme_init(&bme, &hi2c1);
  // Check status - though I don't know if this can
  // be anything other than ok at this point, since it
  // was just initialized...
  // TODO: Implement bme_status_string function
  int bme_status = bme_check_status(&bme);
  {
    if (bme_status == BME68X_ERROR)
    {
      debug_print("Sensor error:" + bme_status);
      // debug_print("Sensor error:" + bme.statusString());
      return BME68X_ERROR;
    }
    else if (bme_status == BME68X_WARNING)
    {
      debug_print("Sensor Warning:" + bme_status);
      // debug_print("Sensor Warning:" + bme.statusString());
    }
  }
  // bme.setTPH (set temp, pressure, humidity config)
  bme_set_TPH_default(&bme);
  // bme.setHeaterProf(int, int) (set heater profile)
  /* Set the heater configuration to 300 deg C for 100ms for Forced mode */
  bme_set_heaterprof(&bme, 300, 100);
      // Initialize bme data struct (here or in loop??)
      // bme.setOpMode(forced_mode_enum) (here or in loop??)

      uint8_t sensor_id;
  bme_read(0xD0, &sensor_id, 4, &hi2c1);
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

/* USER CODE END 4 */
