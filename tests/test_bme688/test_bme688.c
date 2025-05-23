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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_I2C_IsDeviceReady(&hi2c1, BME68X_ADDR, 3, HAL_MAX_DELAY) == HAL_OK)
  {
    debug_print("Sensor is ready\r\n");
  }
  else
  {
    debug_print("Sensor not responding\r\n");
  }

  // Create bme interface struct and initialize it
  bme68x_sensor_t bme;

  // Test direct read and write
  uint8_t sensor_id;
  bme_read_direct(0xD0, &sensor_id, 4, &hi2c1);
  debug_print("Received sensor ID: 0x%X\r\n", sensor_id);

  bme_init(&bme, &hi2c1, &delay_us_timer);
  // Check status, should be 0 for OK
  int bme_status = bme_check_status(&bme);
  {
    if (bme_status == BME68X_ERROR)
    {
      debug_print("Sensor error:" + bme_status);
      return BME68X_ERROR;
    }
    else if (bme_status == BME68X_WARNING)
    {
      debug_print("Sensor Warning:" + bme_status);
    }
  }
  // Set temp, pressure, humidity oversampling configuration
  // Trying with defaults
  bme_set_TPH_default(&bme);
  // Alternatively, could set each to sample only once:
  // bme_set_TPH(&bme, BME68X_OS_1X, BME68X_OS_1X, BME68X_OS_1X);
  // Set the heater configuration to 300 deg C for 100ms for Forced mode
  bme_set_heaterprof(&bme, 300, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    // Set to forced mode, which takes a single sample and returns to sleep mode
    bme_set_opmode(&bme, BME68X_FORCED_MODE);
    /** @todo: May adjust the specific timing function called here, but it should be based on bme_get_meas_dur */
    delay_us_timer(bme_get_meas_dur(&bme, BME68X_SLEEP_MODE), &hi2c1);
    // Fetch data
    int fetch_success = bme_fetch_data(&bme);
    if (fetch_success)
    {
      debug_print("Temperature: %d.%02d°C, ",
                  bme.sensor_data.temperature / 100,
                  (bme.sensor_data.temperature % 100));
      debug_print("Pressure: %d Pa, ", bme.sensor_data.pressure);
      debug_print("Humidity: %d.%03d%%, ",
                  bme.sensor_data.humidity / 1000,
                  (bme.sensor_data.humidity % 1000));
      debug_print("Gas Resistance: %d.%03d kΩ, ",
                  bme.sensor_data.gas_resistance / 1000,
                  (bme.sensor_data.gas_resistance % 1000));
      debug_print("Status: 0x%X\r\n", bme.sensor_data.status);
    }

    HAL_Delay(1000);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
