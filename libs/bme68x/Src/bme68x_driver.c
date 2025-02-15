/*******************************************************************************
 * @file: bme68x_driver.c
 * @version: 0.1.0
 * @sources:
 *   - Bosch BME68x library and Arduino examples
 *     https://github.com/boschsensortec/Bosch-BME68x-Library
 ******************************************************************************/

#include "bme68x_driver.h"

int8_t bme68x_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, I2C_HandleTypeDef *i2c_handle) {
  return 0;
}

/**
 * @brief Function that implements the default I2C read transaction
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to be written to the sensor
 * @param length   : Length of the transfer
 * @param i2c_handle : Pointer to the stm32 I2C peripheral handle
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme68x_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, I2C_HandleTypeDef *i2c_handle) {
  return 0;
}
