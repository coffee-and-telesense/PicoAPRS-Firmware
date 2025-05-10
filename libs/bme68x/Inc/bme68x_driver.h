/*******************************************************************************
 * @file: bme68x_driver.h
 * @brief: Driver for the Bosch ME68x sensor using STM32 HAL.
 *
 * This file provides function prototypes and macros for interfacing
 * with the BME68x sensor using the STM32 HAL I2C interface.
 *
 * @version: 0.1.0
 * @sources:
 *   - Bosch BME68x library and Arduino examples
 *     https://github.com/boschsensortec/Bosch-BME68x-Library
 ******************************************************************************/
#pragma once

/* ========================== INCLUDES ========================== */
#ifdef DEBUG
#include "logging.h"
#endif

/**
 * @brief Include the I2C HAL directly
 * @todo: This will need to be updated for the target MCU (e.g. the U0)
 * @todo: Alternatively, we could include an "i2c.h" file under the assumption that one will be created within the application code.
 */
#include "stm32u0xx_hal.h"
/** @note: As currently written, the Bosch library needs BME68X_DO_NOT_USE_FPU
 * to be set in order to prevent floating point code from being used. This is currently
 * set in the CMakeLists.txt file for this driver.
 */
#include "bme68x_defs.h"
#include "bme68x.h"

/* ========================== MACROS ========================== */

/**
 * @brief Basic error macro
 * @todo: Consider using in conjunction with the common error types macros
 */
#define BME68X_ERROR INT8_C(-1)
#define BME68X_WARNING INT8_C(1)

/**
 * @brief Alias the default address, with a left shift to use expected HAL format
 */
#define BME68X_ADDR (BME68X_I2C_ADDR_HIGH << 1)

/* ========================== TYPE DEFINITIONS ========================== */

/**
 * @struct bme68x_sensor_t
 * @brief Structure to store necessary configuration and data to interact with the sensor over I2C.
 */
typedef struct
{
  /** Stores the BME68x sensor APIs error code after execution. */
  int8_t status;

  /** Pointer to the I2C handle (e.g., `&hi2c1`). */
  I2C_HandleTypeDef *i2c_handle;

  /** BME68X device structure. See `bme68x_defs.h` for details. */
  struct bme68x_dev device;

  /** Sensor configuration settings, including oversampling and filter coefficients. */
  struct bme68x_conf conf;

  /** Heater configuration settings. See `bme68x_defs.h` for details. */
  struct bme68x_heatr_conf heatr_conf;

  /** Structure to store sensor measurement data. */
  /** @note: Since we're not using floating point, the data will be as follows:
   *  - Temperature in degrees celsius x100
   *  - Pressure in Pascal
   *  - Humidity in % relative humidity x1000
   *  - Gas resistance in Ohms
   */
  struct bme68x_data sensor_data;

  /** Last operation mode used by the sensor. */
  uint8_t last_opmode;
} bme68x_sensor_t;

/* ========================== FUNCTION PROTOTYPES ========================== */

/**
 * @brief Initializes the BME68x sensor.
 *
 * Configures the sensor with default settings.
 *
 * @param[in,out] bme Pointer to newly initialized bme68x sensor interface
 * @param[in] i2c_handle Pointer to I2C handle.
 */
void bme_init(bme68x_sensor_t *bme, I2C_HandleTypeDef *i2c_handle, bme68x_delay_us_fptr_t delay_fn);

/**
 * @brief Returns basic status check
 *
 * Checks and returns current bme instance status
 *
 * @param[in] bme Pointer to bme68x sensor interface
 * @return 0 for status OK, 1 for warning, -1 for error
 * @todo: Is this really a necessary or helpful function?
 */
int8_t bme_check_status(bme68x_sensor_t *bme);

/**
 * @brief Set the Temperature, Pressure and Humidity over-sampling using default values.
 *
 * Uses as values BME68X_OS_2X, BME68X_OS_16X, and BME68X_OS_1X for os_temp, os_pres,
 * and os_hum, respectively, for the function bme_set_TPH.
 *
 * @param[in] bme Pointer to bme68x sensor interface
 */
void bme_set_TPH_default(bme68x_sensor_t *bme);

/**
 * @brief Set the Temperature, Pressure and Humidity over-sampling.
 *
 * Sets bme struct members os_temp, os_pres, and os_hum, and calls bme68x_set_conf,
 * which sets the number of measurments to perform. See bme68x_defs.h for macro definitions.
 *
 * @param[in] bme Pointer to bme68x sensor interface
 * @param[in] os_temp Number of measurements to perform, BME68X_OS_NONE to BME68X_OS_16X
 * @param[in] os_pres Number of measurements to perform, BME68X_OS_NONE to BME68X_OS_16X
 * @param[in] os_hum Number of measurements to perform, BME68X_OS_NONE to BME68X_OS_16X
 */
void bme_set_TPH(bme68x_sensor_t *bme, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum);

/**
 * @brief Set the heater profile for Forced mode
 *
 * Determines the gas configuration of the sensor; modify to target
 * different types of gases.
 *
 * @param[in] bme Pointer to bme68x sensor interface
 * @param[in] temp Heater temperature in degree Celsius
 * @param[in] dur Heating duration in milliseconds
 */
void bme_set_heaterprof(bme68x_sensor_t *bme, uint16_t temp, uint16_t dur);

/**
 * @brief Set the operation mode
 *
 * Set the desired operation mode. Currently supports sleep mode and forced mode for
 * a single, low-power measurment which may automatically return to sleep mode.
 * The gas sensor heater only operates during measurement in this mode.
 *
 * @param[in] bme Pointer to bme68x sensor interface
 * @param[in] opmode BME68X_SLEEP_MODE, BME68X_FORCED_MODE
 */
void bme_set_opmode(bme68x_sensor_t *bme, uint8_t opmode);

/**
 * @brief Fetch data from the sensor into the struct's sensor_data buffer
 *
 * Calls the bme68x_get_data function, which reads the pressure, temperature,
 * and humidity and gas data from the sensor, compensates the data, and
 * stores it in the bme->sensor_data struct member.
 *
 * @param[in] bme Pointer to bme68x sensor interface
 * @return Number of new data fields
 */
uint8_t bme_fetch_data(bme68x_sensor_t *bme);

/**
 * @brief Get the measurement duration in microseconds
 *
 * Calls bme68x_get_meas_dur, which calculates the total measurement duration
 * based on the total number of samples to be taken, as determined by the oversampling
 * for each value; a wake-up duration of 1ms is added.
 *
 * @param[in] bme Pointer to bme68x sensor interface
 * @param[in] opmode Operation mode of the sensor. Attempts to use the last one if nothing is set
 * @return Temperature, Pressure, Humidity measurement time in microseconds
 */
uint32_t bme_get_meas_dur(bme68x_sensor_t *bme, uint8_t opmode);

/**
 * @brief Implements the default I2C write transaction
 *
 * Sets the write function pointer on the bme68x_dev device struct.
 * Used throughout the Bosch library to write to the device.
 *
 * @param[in] reg_addr Register address of the sensor
 * @param[in] reg_data Pointer to the data to be written to the sensor
 * @param[in] length Length of the transfer
 * @param[in] i2c_handle Pointer to the stm32 I2C peripheral handle
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 * @brief Implements the default I2C read transaction
 *
 * Sets the read function pointer on the bme68x_dev device struct.
 * Used throughout the Bosch library to read from the device.
 *
 * @param[in] reg_addr Register address of the sensor
 * @param[in] reg_data Pointer to the data to write the sensor value to
 * @param[in] length Length of the transfer
 * @param[in] i2c_handle Pointer to the stm32 I2C peripheral handle
 * @return 0 if successful, non-zero otherwise
 */
int8_t bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/**
 * @example main.c
 *
 * Example usage of the BME68x sensor driver.
 * @note: Requires HAL peripheral drivers and initialization, specifically for i2c.
 * @note: Assumes an I2C_HandleTypeDef hi2c1 instance exists.
 * @note: Requires a microsecond delay function implemented in the application.
 *
 * @code
 * #include "bme68x_driver.h"
 *
 * int main(void) {
 *     bme68x_sensor_t bme;
 *     bme_init(&bme, &hi2c1, &delay_us_timer);
 *     bme_set_TPH_default(&bme);
 *     bme_set_heaterprof(&bme, 300, 100);
 *     while (1)
 *     {
 *         bme_set_opmode(&bme, BME68X_FORCED_MODE);
 *         // @todo: May adjust the specific timing function called here, but it should be based on bme_get_meas_dur
 *         delay_us_timer(bme_get_meas_dur(&bme, BME68X_SLEEP_MODE), &hi2c1);
 *         int fetch_success = bme_fetch_data(&bme);
 *         if (fetch_success) {
 *           debug_print("%d, ", bme.sensor_data.temperature);
 *           debug_print("%d, ", bme.sensor_data.pressure);
 *           debug_print("%d, ", bme.sensor_data.humidity);
 *           debug_print("%d, ", bme.sensor_data.gas_resistance);
 *           debug_print("%X, \r\n", bme.sensor_data.status);
 *         }
 *     }
 * }
 * @endcode
 */
