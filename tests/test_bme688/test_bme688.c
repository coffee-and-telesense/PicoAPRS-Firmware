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
 * BME68X_SLEEP_MODE     // No measurements
 * BME68X_FORCED_MODE    // One-shot measurement (best for low-power)
 * BME68X_PARALLEL_MODE  // Continuous measurement using heater profiles
 * BME68X_SEQUENTIAL_MODE// Like parallel but runs profiles in sequence
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
#include <math.h>
#include <stdlib.h>

#define BME68X_ADDR (0x77 << 1)  // 0xEE
#define SEALEVEL_PRESSURE 101325
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* ================================ */
/*       BME68x Sensor Function     */
/* ================================ */
/* --- i2c scan for connected sensors --- */
void i2c_scan(void) {
  printf("Scanning I2C bus...\r\n");
  for (uint8_t addr = 1; addr < 127; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 100) == HAL_OK) {
          char buffer[64];
          snprintf(buffer, sizeof(buffer), "Found device at 0x%02X\r\n", addr);
          printf(buffer);
      }
  }
  printf("Done scanning.\r\n");
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  debug_print("System Initialization Complete!\r\n");
  if (HAL_I2C_IsDeviceReady(&hi2c1, BME68X_ADDR, 3, HAL_MAX_DELAY) == HAL_OK)
  {
    debug_print("Sensor is ready\r\n");
  }
  else
  {
    debug_print("Sensor not responding\r\n");
  }
  i2c_scan();
  debug_print("Reading BME\r\n");
  static bme68x_sensor_t bme;               // your “instance” of the sensor
  bme_init(&bme, &hi2c1, &delay_us_timer);  // wire it up to hi2c1 and your delay fn

  // Quick status check
  if (bme_check_status(&bme) != BME68X_OK) {
    debug_print("BME68x init failed!\r\n");
      return;
  }
  else {
    debug_print("BME68x init success!\n");
  }

  // Configure oversampling: T×2, P×16, H×1 (same as the Arduino defaults)
  bme_set_TPH_default(&bme);

  // Set up the heater for a single forced‐mode cycle (300 °C for 100 ms)
  bme_set_heaterprof(&bme, 300, 100);

  while(1){
  // Trigger one forced measurement
  bme_set_opmode(&bme, BME68X_FORCED_MODE);

  // Wait the required duration (driver gives you µs)
  uint32_t wait_us = bme_get_meas_dur(&bme, BME68X_FORCED_MODE);
  HAL_Delay((wait_us + 999) / 1000);  // round up to ms

  // Fetch the data
  uint8_t n_fields = bme_fetch_data(&bme);
  if (n_fields == 0) {
    debug_print("No new data from BME68x\n");
      return;
  }

  // it still doesn't print floats
  //  float T = bme.sensor_data.temperature;     // °C
  //  float P = bme.sensor_data.pressure;        // Pa
  //  float H = bme.sensor_data.humidity;        // %RH (or %RH×1000—check your API)
  //  float G = bme.sensor_data.gas_resistance;  // Ω

  // // If humidity is in ‰ (x1000), divide by 1000.0f to get percent:
  // printf("BME68x: T=%.2f °C, P=%.0f Pa, H=%.3f %%, G=%.0f Ω\r\n",
  //        T,
  //        P,
  //        H / 1000.0f,
  //        G);

  // but ints work
  int32_t rawT = bme.sensor_data.temperature;     // e.g. 2236  => 22.36 °C
  int32_t rawP = bme.sensor_data.pressure;        // e.g. 101325
  int32_t rawH = bme.sensor_data.humidity;        // e.g. 45123 => 45.123 %
  int32_t rawG = bme.sensor_data.gas_resistance;  // e.g. 12000 Ω

  //the whole altitude thing is weird. it works better outside
  //I'll leave the code in here since it's not breaking anything anyways.
  float altitude = 44330.0f * (1.0f - powf((float)rawP / SEALEVEL_PRESSURE, 0.1902949f));
  // round to nearest meter:
  long alt_int = (long)lroundf(altitude);

  debug_print("BME68x: T=%ld.%02ld°C, P=%ldPa, H=%ld.%02ld%%, G=%ldΩ, Alt=%ldm\n",
         rawT / 100, abs(rawT % 100),
         rawP,
         rawH / 1000, abs(rawH % 1000),
         rawG,
         alt_int);
  HAL_Delay(6000);
  debug_print("End of code\n");
  }
  return 0;
}