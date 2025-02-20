/*******************************************************************************
 * @file: bme68x_driver.h
 * @version: 0.1.0
 * @sources:
 *   - Bosch BME68x library and Arduino examples
 *     https://github.com/boschsensortec/Bosch-BME68x-Library
 ******************************************************************************/
#pragma once

/********************************************************* */
/*!             Header includes                           */
/********************************************************* */

#ifdef DEBUG
#include "logging.h"
#endif
// TODO: Include only the target MCU family HAL for specific peripherals
// or the project i2c.h?
#include "stm32l4xx_hal_i2c.h"
// Alternatively, include the CubeMX generated i2c.h for the test project
// #include "i2c.h"
// #include <string.h>
// #include <stdbool.h>

/********************************************************* */
/*!               Common Macros                           */
/********************************************************* */

// Ensure we use fixed point in the Bosch library
// This can be removed when / if the Bosch library is
// reduced to only the necessary functionality
#define BME68X_DO_NOT_USE_FPU

#include "bme68x_defs.h"
#include "bme68x.h"

// TODO: Use error types from common?
#define BME68X_ERROR INT8_C(-1)
#define BME68X_WARNING INT8_C(1)

// Alias default address
#define BME68X_ADDR (BME68X_I2C_ADDR_HIGH << 1) // HAL expects 8-bit address (shifted left)

// TODO: Use macro to construct bme-specific error codes

typedef struct
{
  int8_t status;                 // Stores the BME68x sensor APIs error code after an execution
  I2C_HandleTypeDef *i2c_handle; // I2C handle (e.g. *hi2c1)
  struct bme68x_dev device;             // BME68X device (see bme68x_defs.h)
  struct bme68x_conf conf;              // Configuration settings (see bme68x_defs.h, includes oversampling and filter coefficient settings)
  struct bme68x_heatr_conf heatr_conf;  // Heater configuration (see bme68x_defs.h)
  struct bme68x_data sensor_data;       // Sensor data storage (see bme68x_defs.h, starting with a single value)
  // bme68x_data sensor_data[3];   // Sensor data storage (see bme68x_defs.h, assuming max of 3)
  // uint8_t n_fields, i_fields;   // Used for parallel mode
  // uint8_t last_op_mode;         // Last operation mode used
} bme68x_sensor_t;

/********************************************************* */
/*!               Function Pointers                       */
/********************************************************* */

void bme_init(bme68x_sensor_t *bme, I2C_HandleTypeDef *i2c_handle);
int8_t bme_check_status(bme68x_sensor_t *bme);
void bme_set_TPH_default(bme68x_sensor_t *bme);
void bme_set_TPH(bme68x_sensor_t *bme, uint8_t osTemp, uint8_t osPres, uint8_t osHum);

/**
 * @brief Function that implements the default I2C write transaction
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param i2c_handle : Pointer to the stm32 I2C peripheral handle
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 * @brief Function that implements the default I2C read transaction
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to write the sensor value to
 * @param length   : Length of the transfer
 * @param i2c_handle : Pointer to the stm32 I2C peripheral handle
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
