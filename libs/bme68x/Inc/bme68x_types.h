#pragma once

#include <stdint.h>

/**
 * @brief BME68x status and error codes
 */
typedef enum {
    // General status
    BME68X_OK = 0,
    BME68X_ERROR = -1,
    BME68X_WARNING = 1,

    // Interface-related errors
    BME68X_I2C_READ_FAILED = 10,
    BME68X_I2C_WRITE_FAILED = 11,

    // Parameter & pointer validation
    BME68X_NULL_PTR = 20,
    BME68X_INVALID_ARG = 21,

    // Driver-specific setup/init errors
    BME68X_INIT_FAILED = 30,
    BME68X_SET_CONF_FAILED = 31,
    BME68X_GET_CONF_FAILED = 32,
    BME68X_SET_HEATER_FAILED = 33,
    BME68X_SET_MODE_FAILED = 34,
    BME68X_GET_DATA_FAILED = 35,

    // State-specific errors
    BME68X_NOT_INITIALIZED = 40

} bme_status_e;
