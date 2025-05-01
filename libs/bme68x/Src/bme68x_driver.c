/*******************************************************************************
 * @file: bme68x_driver.c
 * @brief: Driver for the BME68x sensor using STM32 HAL.
 *
 * This file contains the implementation of functions for configuring
 * and reading data from the BME68x sensor using STM32 HAL.
 *
 * @version: 0.1.0
 * @sources:
 *   - Bosch BME68x library and Arduino examples
 *     https://github.com/boschsensortec/Bosch-BME68x-Library
 ******************************************************************************/

#include "bme68x_driver.h"

#include <string.h>
/* ========================== FUNCTION IMPLEMENTATIONS ========================== */

/** Initializes the BME68x sensor interface */
void bme_init(bme68x_sensor_t *bme, I2C_HandleTypeDef *i2c_handle, bme68x_delay_us_fptr_t delay_fn) {
    if (bme == NULL) {
        return;
    }

    // Zero-initialize all members
    memset(bme, 0, sizeof(bme68x_sensor_t));

    // Assign the I2C handle
    bme->i2c_handle = i2c_handle;

    //
    // Set default values
    //
    bme->device.intf_ptr = i2c_handle;
    /** @todo (maybe) Assign variant ID on bme struct */
    bme->device.read = &bme_read;
    bme->device.write = &bme_write;
    bme->device.delay_us = delay_fn;
    bme->status = BME68X_OK;
    // Typical ambient temperature in Celsius
    bme->device.amb_temp = 25;
    // Assume sensor starts in sleep mode
    bme->last_opmode = BME68X_SLEEP_MODE;

    bme68x_init(&bme->device);
}

/** Returns basic status check */
int8_t bme_check_status(bme68x_sensor_t *bme) {
    if (bme->status < BME68X_OK) {
        return BME68X_ERROR;
    } else if (bme->status > BME68X_OK) {
        return BME68X_WARNING;
    } else {
        return BME68X_OK;
    }
}

/** Set the Temperature, Pressure and Humidity over-sampling using default values. */
void bme_set_TPH_default(bme68x_sensor_t *bme) {
    bme_set_TPH(bme, BME68X_OS_2X, BME68X_OS_16X, BME68X_OS_1X);
}

/** Set the Temperature, Pressure and Humidity over-sampling.*/
void bme_set_TPH(bme68x_sensor_t *bme, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum) {
    bme->status = bme68x_get_conf(&bme->conf, &bme->device);

    if (bme->status == BME68X_OK) {
        bme->conf.os_hum = os_hum;
        bme->conf.os_temp = os_temp;
        bme->conf.os_pres = os_pres;

        bme->status = bme68x_set_conf(&bme->conf, &bme->device);
    }
}

/** Set the heater profile for Forced mode */
void bme_set_heaterprof(bme68x_sensor_t *bme, uint16_t temp, uint16_t dur) {
    bme->heatr_conf.enable = BME68X_ENABLE;
    bme->heatr_conf.heatr_temp = temp;
    bme->heatr_conf.heatr_dur = dur;

    bme->status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme->heatr_conf, &bme->device);
}

/** Set the operation mode */
void bme_set_opmode(bme68x_sensor_t *bme, uint8_t opmode) {
    bme->status = bme68x_set_op_mode(opmode, &bme->device);
    if ((bme->status == BME68X_OK) && (opmode != BME68X_SLEEP_MODE))
        bme->last_opmode = opmode;
}

/** Fetch data from the sensor into the struct's sensor_data buffer */
uint8_t bme_fetch_data(bme68x_sensor_t *bme) {
    uint8_t n_fields = 0;
    bme->status = bme68x_get_data(bme->last_opmode, &bme->sensor_data, &n_fields, &bme->device);
    return n_fields;
}

/** Get the measurement duration in microseconds*/
uint32_t bme_get_meas_dur(bme68x_sensor_t *bme, uint8_t opmode) {
    if (opmode == BME68X_SLEEP_MODE)
        opmode = bme->last_opmode;

    return bme68x_get_meas_dur(opmode, &bme->conf, &bme->device);
}

/** Implements the default microsecond delay callback */
int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  // Cast the I2C handle back to the correct type
  I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)intf_ptr;

  // For multi-byte writes, we need pairs of register addresses and data,
  // since the sensor does *not* auto-increment for multi-byte writes.
  // Create a buffer with enough space for all register/data pairs
  uint8_t buffer[length * 2];

  // Fill the buffer with register/data pairs
  /** @todo: Determine max size needed for this buffer */
  for (uint16_t i = 0; i < length; i++)
  {
    buffer[i * 2] = reg_addr + i;
    buffer[i * 2 + 1] = reg_data[i];
  }

  // Transmit all register/data pairs in a single transaction
  if (HAL_I2C_Master_Transmit(i2c_handle, BME68X_ADDR, buffer, length * 2, HAL_MAX_DELAY) != HAL_OK)
  {
    // Error in transmission
    return -1;
  }

  // Return 0 on success
  return 0;
}

/** Implements the default microsecond delay callback */
int8_t bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  // Cast the I2C handle back to the correct type
  I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)intf_ptr;

  /** @todo: Consider using HAL_I2C_Mem_Read in a single transaction */
  // Send the register address (1 byte)
  if (HAL_I2C_Master_Transmit(i2c_handle, BME68X_ADDR, &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    // Error in transmitting register address
    return -1;
  }

  // Read the requested number of bytes
  // The sensor auto-increments for I2C reads
  if (HAL_I2C_Master_Receive(i2c_handle, BME68X_ADDR, reg_data, length, HAL_MAX_DELAY) != HAL_OK)
  {
    // Error in reading data
    return -1;
  }
