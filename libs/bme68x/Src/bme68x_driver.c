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
    if (bme == NULL || i2c_handle == NULL || delay_fn == NULL) {
        if (bme) bme->status = BME68X_E_NULL_PTR;
        return;
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
}

/** Returns basic status check */
int8_t bme_check_status(bme68x_sensor_t *bme) {
    if (bme == NULL) {
        return BME68X_E_NULL_PTR;
    }
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

/** Set the Temperature, Pressure and Humidity over-sampling. */
void bme_set_TPH(bme68x_sensor_t *bme, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum) {
    if (bme == NULL) {
        return;
    }

    bme->status = bme68x_get_conf(&bme->conf, &bme->device);
    if (bme->status == BME68X_OK) {
        bme->conf.os_hum = os_hum;
        bme->conf.os_temp = os_temp;
        bme->conf.os_pres = os_pres;

        bme->status = bme68x_set_conf(&bme->conf, &bme->device);
    } else {
        bme->status = BME68X_E_COM_FAIL;
    }
}

/** Set the heater profile for Forced mode */
void bme_set_heaterprof(bme68x_sensor_t *bme, uint16_t temp, uint16_t dur) {
    if (bme == NULL) {
        return;
    }

    bme->heatr_conf.enable = BME68X_ENABLE;
    bme->heatr_conf.heatr_temp = temp;
    bme->heatr_conf.heatr_dur = dur;

    bme->status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme->heatr_conf, &bme->device);
    if (bme->status != BME68X_OK) {
        bme->status = BME68X_INIT_SENSOR_ERROR;
    }
}

/** Set the operation mode */
void bme_set_opmode(bme68x_sensor_t *bme, uint8_t opmode) {
    if (bme == NULL) {
        bme->status = BME68X_E_COM_FAIL;
        return;
    }

    bme->status = bme68x_set_op_mode(opmode, &bme->device);
    if ((bme->status == BME68X_OK) && (opmode != BME68X_SLEEP_MODE))
        bme->last_opmode = opmode;
}

/** Fetch data from the sensor into the struct's sensor_data buffer */
uint8_t bme_fetch_data(bme68x_sensor_t *bme) {
    uint8_t n_fields = 0;

    if (bme == NULL) {
        return 0;
    }

    bme->status = bme68x_get_data(bme->last_opmode, &bme->sensor_data, &n_fields, &bme->device);
    if (bme->status != BME68X_OK) {
        bme->status = BME68X_E_COM_FAIL;
    }

    return n_fields;
}

/** Get the measurement duration in microseconds */
uint32_t bme_get_meas_dur(bme68x_sensor_t *bme, uint8_t opmode) {
    if (bme == NULL) {
        return 0;
    }

    if (opmode == BME68X_SLEEP_MODE) {
        opmode = bme->last_opmode;
    }

    return bme68x_get_meas_dur(opmode, &bme->conf, &bme->device);
}

/** I2C Write wrapper */
int8_t bme_write(uint8_t reg_addr,
                 const uint8_t *reg_data,
                 uint32_t length,
                 void *intf_ptr) {
    HAL_StatusTypeDef r = HAL_I2C_Mem_Write(
        (I2C_HandleTypeDef *)intf_ptr,
        BME68X_ADDR,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)reg_data,
        length,
        HAL_MAX_DELAY);
    return (r == HAL_OK ? BME68X_OK : BME68X_I2C_ERROR);
}

/** I2C Read wrapper */
int8_t bme_read(uint8_t reg_addr,
                uint8_t *reg_data,
                uint32_t length,
                void *intf_ptr) {
    HAL_StatusTypeDef r = HAL_I2C_Mem_Read(
        (I2C_HandleTypeDef *)intf_ptr,
        BME68X_ADDR,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        reg_data,
        length,
        HAL_MAX_DELAY);
    return (r == HAL_OK ? BME68X_OK : BME68X_I2C_ERROR);
}

/** Optional: String representation of error codes for debugging */
const char* bme_error_string(int8_t status) {
    switch (status) {
        case BME68X_OK: return "OK2";
        case BME68X_E_NULL_PTR: return "Null pointer error";
        case BME68X_E_COM_FAIL: return "Communication failure";
        case BME68X_E_DEV_NOT_FOUND: return "Device not found";
        case BME68X_E_INVALID_LENGTH: return "Invalid length parameter";
        case BME68X_E_SELF_TEST: return "Self test failed";
        case BME68X_ERROR: return "Generic sensor error";
        case BME68X_SENSOR_ERROR: return "Sensor internal error";
        case BME68X_INIT_SENSOR_ERROR: return "Sensor initialization error";
        case BME68X_WARNING: return "Warning";
        case BME68X_TEMP: return "Temperature error";
        case BME68X_PRES: return "Pressure error";
        case BME68X_GAS: return "Gas resistance error";
        case BME68X_HUMID: return "Humidity error";
        case BME68X_I2C_ERROR: return "I2C error";
        default: return "Unknown error";
    }
}
