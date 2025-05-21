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

/**
 * @brief Initializes the BME68x sensor interface with default configuration.
 * 
 * This function sets up the I2C handle, function pointers, and default values,
 * and calls bme68x_init from the Bosch driver. It updates the bme->status.
 * 
 * @param bme Pointer to sensor struct
 * @param i2c_handle Pointer to HAL I2C handle
 * @param delay_fn Delay function with microsecond resolution
 * @return bme_status_e Status of initialization
 */
bme_status_e bme_init(bme68x_sensor_t *bme, I2C_HandleTypeDef *i2c_handle, bme68x_delay_us_fptr_t delay_fn) {
    if (!bme || !i2c_handle || !delay_fn) {
        return BME68X_NULL_PTR;
    }

    memset(bme, 0, sizeof(bme68x_sensor_t));

    bme->i2c_handle = i2c_handle;
    bme->device.intf_ptr = i2c_handle;
    bme->device.read = &bme_read;
    bme->device.write = &bme_write;
    bme->device.delay_us = delay_fn;
    bme->device.amb_temp = 25;
    bme->last_opmode = BME68X_SLEEP_MODE;

    bme->status = bme68x_init(&bme->device);
    if (bme->status != BME68X_OK) {
        return BME68X_INIT_FAILED;
    }

    return BME68X_OK;
}

/**
 * @brief Checks current status of the sensor interface and classifies it.
 * 
 * @param bme Pointer to sensor struct
 * @return bme_status_e OK, WARNING, or ERROR based on status value
 */
bme_status_e bme_check_status(bme68x_sensor_t *bme) {
    if (!bme) return BME68X_NULL_PTR;

    if (bme->status < BME68X_OK) {
        return BME68X_ERROR;
    } else if (bme->status > BME68X_OK) {
        return BME68X_WARNING;
    } else {
        return BME68X_OK;
    }
}

/**
 * @brief Sets temperature, pressure, and humidity oversampling to default values.
 * 
 * Default: Temperature = 2x, Pressure = 16x, Humidity = 1x
 * 
 * @param bme Pointer to sensor struct
 * @return bme_status_e Status after applying configuration
 */
bme_status_e bme_set_TPH_default(bme68x_sensor_t *bme) {
    return bme_set_TPH(bme, BME68X_OS_2X, BME68X_OS_16X, BME68X_OS_1X);
}

/**
 * @brief Sets temperature, pressure, and humidity oversampling to custom values.
 * 
 * @param bme Pointer to sensor struct
 * @param os_temp Oversampling for temperature
 * @param os_pres Oversampling for pressure
 * @param os_hum Oversampling for humidity
 * @return bme_status_e Status after setting configuration
 */
bme_status_e bme_set_TPH(bme68x_sensor_t *bme, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum) {
    if (!bme) return BME68X_NULL_PTR;

    bme->status = bme68x_get_conf(&bme->conf, &bme->device);
    if (bme->status != BME68X_OK) return BME68X_GET_CONF_FAILED;

    bme->conf.os_hum = os_hum;
    bme->conf.os_temp = os_temp;
    bme->conf.os_pres = os_pres;

    bme->status = bme68x_set_conf(&bme->conf, &bme->device);
    if (bme->status != BME68X_OK) return BME68X_SET_CONF_FAILED;

    return BME68X_OK;
}

/**
 * @brief Sets heater profile for forced mode operation.
 * 
 * @param bme Pointer to sensor struct
 * @param temp Target heater temperature in °C
 * @param dur Heater duration in milliseconds
 * @return bme_status_e Status after setting heater config
 */
bme_status_e bme_set_heaterprof(bme68x_sensor_t *bme, uint16_t temp, uint16_t dur) {
    if (!bme) return BME68X_NULL_PTR;

    bme->heatr_conf.enable = BME68X_ENABLE;
    bme->heatr_conf.heatr_temp = temp;
    bme->heatr_conf.heatr_dur = dur;

    bme->status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme->heatr_conf, &bme->device);
    if (bme->status != BME68X_OK) return BME68X_SET_HEATER_FAILED;

    return BME68X_OK;
}

/**
 * @brief Sets the sensor's operation mode (e.g., sleep or forced).
 * 
 * @param bme Pointer to sensor struct
 * @param opmode Operation mode constant
 * @return bme_status_e Status after setting mode
 */
bme_status_e bme_set_opmode(bme68x_sensor_t *bme, uint8_t opmode) {
    if (!bme) return BME68X_NULL_PTR;

    bme->status = bme68x_set_op_mode(opmode, &bme->device);
    if (bme->status != BME68X_OK) return BME68X_SET_MODE_FAILED;

    if (opmode != BME68X_SLEEP_MODE) {
        bme->last_opmode = opmode;
    }
    return BME68X_OK;
}

/**
 * @brief Fetches data from the sensor.
 * 
 * @param bme Pointer to sensor struct
 * @param n_fields Pointer to field count variable (output)
 * @return bme_status_e Status of data fetch
 */
bme_status_e bme_fetch_data(bme68x_sensor_t *bme, uint8_t *n_fields) {
    if (!bme || !n_fields) return BME68X_NULL_PTR;

    *n_fields = 0;
    bme->status = bme68x_get_data(bme->last_opmode, &bme->sensor_data, n_fields, &bme->device);
    if (bme->status != BME68X_OK) return BME68X_GET_DATA_FAILED;

    return BME68X_OK;
}

/**
 * @brief Returns the duration of a single measurement (in microseconds).
 * 
 * @param bme Pointer to sensor struct
 * @param opmode Operation mode to check for
 * @return uint32_t Measurement duration in microseconds, 0 on error
 */
uint32_t bme_get_meas_dur(bme68x_sensor_t *bme, uint8_t opmode) {
    if (!bme) return 0;

    if (opmode == BME68X_SLEEP_MODE) {
        opmode = bme->last_opmode;
    }

    return bme68x_get_meas_dur(opmode, &bme->conf, &bme->device);
}

/**
 * @brief Low-level I2C write function used by Bosch driver.
 * 
 * @param reg_addr Register address to write to
 * @param reg_data Pointer to data buffer
 * @param length Length of data
 * @param intf_ptr Pointer to I2C handle
 * @return int8_t BME68X_OK or BME680_I2C_WRITE_FAILED
 */
int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    if (!reg_data || !intf_ptr) return BME68X_NULL_PTR;

    HAL_StatusTypeDef r = HAL_I2C_Mem_Write(
        (I2C_HandleTypeDef *)intf_ptr,
        BME68X_ADDR,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)reg_data,
        length,
        HAL_MAX_DELAY);

    return (r == HAL_OK ? BME68X_OK : BME68X_I2C_WRITE_FAILED);
}

/**
 * @brief Low-level I2C read function used by Bosch driver.
 * 
 * @param reg_addr Register address to read from
 * @param reg_data Pointer to receive buffer
 * @param length Number of bytes to read
 * @param intf_ptr Pointer to I2C handle
 * @return int8_t BME68X_OK or BME680_I2C_READ_FAILED
 */
int8_t bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    if (!reg_data || !intf_ptr) return BME68X_NULL_PTR;

    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(
        (I2C_HandleTypeDef *)intf_ptr,
        BME68X_ADDR,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        reg_data,
        length,
        HAL_MAX_DELAY);

    return (r == HAL_OK ? BME68X_OK : BME68X_I2C_READ_FAILED);
}
