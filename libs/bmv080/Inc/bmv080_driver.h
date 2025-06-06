/*******************************************************************************
 * @file: bmv080_driver.h
 * @brief: Driver for the Bosch BMV080 sensor using STM32 HAL.
 *
 * This file provides function prototypes and macros for interfacing
 * with the BMV080 sensor using the STM32 HAL I2C interface.
 *
 * @version: 0.1.0
 ******************************************************************************/
#pragma once

/* ========================== INCLUDES ========================== */
#ifdef DEBUG
#include "logging.h"
#endif

#include "stm32u0xx_hal.h"
#include "bmv080_defs.h"
#include "bmv080.h"

#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <string.h> // For memset

/* ========================== MACROS ========================== */
/** @note: The datasheet states that the default I2C address is 0x57, but the Bosch
 * example code uses 0x54 as the default. User should confirm the address and override
 * as needed.
 */
#define BMV080_I2C_ADDRESS 0x57

/* ========================== TYPE DEFINITIONS ========================== */
/**
 * @struct bmv080_sensor_t
 * @brief Structure representing a complete BMV080 sensor instance.
 *
 * Encapsulates sensor handles, current status, output data, and data-ready flag.
 * Intended to simplify the use of the BMV080 sensor through a wrapper layer.
 */
typedef struct
{
  bmv080_handle_t handle;             /**< Internal driver handle, initialized to NULL */
  bmv080_status_code_t status;        /**< Latest status code from BMV080 operations */
  bmv080_sercom_handle_t i2c_handle;  /**< I2C communication interface */
  bmv080_output_t output;             /**< Most recent sensor output structure */
  bool data_available;                /**< Indicates whether new sensor data is available */
} bmv080_sensor_t;

/**
 * @struct bmv080_fixed_t
 * @brief Fixed-point representation of BMV080 sensor output for transmission.
 *
 * Encodes selected fields from the BMV080 sensor output structure using fixed-point formats.
 * PM values are scaled from 0–1000 µg/m³ into uint16_t (0–65535). Runtime is stored with
 * 0.01-second resolution, and flags indicate obstruction and range validity.
 */
typedef struct
{
  uint16_t pm1;                 /**< PM1 mass concentration in fixed-point format (0–65535 maps to 0–1000 µg/m³) */
  uint16_t pm2_5;               /**< PM2.5 mass concentration in fixed-point format (0–65535 maps to 0–1000 µg/m³) */
  uint16_t pm10;                /**< PM10 mass concentration in fixed-point format (0–65535 maps to 0–1000 µg/m³) */
  uint16_t runtime_in_0_01_sec; /**< Runtime since measurement start, stored in 0.01-second resolution (max ~655 s) */
  uint8_t flags;                /**< Bitfield: [0] = obstructed, [1] = outside measurement range */
} bmv080_fixed_t;

/* ========================== FUNCTION PROTOTYPES ========================== */
/**
 * @brief Initializes a BMV080 sensor structure with default values and I2C interface.
 *
 * Sets the sensor handle to NULL, status to E_BMV080_OK, clears the output data,
 * and sets the data availability flag to false. Assigns the provided I2C handle
 * to the communication interface field of the sensor.
 *
 * @param[in,out] sensor Pointer to the bmv080_sensor_t instance to initialize
 * @param[in] i2c_handle Pointer to the I2C handle used for BMV080 communication
 */
void bmv080_init(bmv080_sensor_t *sensor, I2C_HandleTypeDef *i2c_handle);

/**
 * @brief Get the current system time in milliseconds
 *
 * Used by the BMV080 driver for timing in duty cycle mode.
 *
 * @return Current tick count in milliseconds
 */
uint32_t bmv080_get_tick_ms(void);

/**
 * @brief Configure the sensor's duty cycling period
 *
 * Resets the sensor and applies a new duty cycle interval in seconds.
 *
 * @param[in,out] sensor Pointer to initialized sensor struct
 * @param[in] period_s Desired duty cycling period in seconds (e.g., 10, 20)
 * @return Status code from BMV080 API
 */
bmv080_status_code_t bmv080_configure_duty_cycle(bmv080_sensor_t *sensor, uint16_t period_s);

/**
 * @brief Start a BMV080 measurement in duty cycling mode
 *
 * Uses the internal system tick to support time-based cycling.
 *
 * @param[in,out] sensor Pointer to initialized sensor struct
 * @return Status code from BMV080 API
 */
bmv080_status_code_t bmv080_start_duty_cycle(bmv080_sensor_t *sensor);

/**
 * @brief Poll the sensor for new measurement data
 *
 * Should be called repeatedly in a loop. If new data is available,
 * updates `sensor->output` and sets `sensor->data_available = true`.
 *
 * @param[in,out] sensor Pointer to initialized sensor struct
 * @return Status code from BMV080 API
 */
bmv080_status_code_t bmv080_poll(bmv080_sensor_t *sensor);

/**
 * @brief Stop the current measurement session
 *
 * @param[in,out] sensor Pointer to initialized sensor struct
 * @return Status code from BMV080 API
 */
bmv080_status_code_t bmv080_stop(bmv080_sensor_t *sensor);

/**
 * @brief Print a sensor output record over UART
 *
 * @param[in] output Pointer to a valid BMV080 output structure
 */
void bmv080_print_output(const bmv080_output_t *output);

/**
 * @brief Implements the default I2C read transaction
 *
 * Sets the read function callback pointer on the bmv080 device struct.
 * Used throughout the Bosch library to read from the device. See the
 * typedef for bmv080_callback_read_t in bmv080_defs.h for additional details.
 *
 * @param[in] handle Handle for the I2C interface
 * @param[in] header Header information for the following _payload_.
 * @param[in] payload Payload to be read consisting of 16 bit words.
 * @param[in] payload_length Lumber of _payload_ elements to be read.
 * @return 0 if successful, otherwise the return value is an externally defined error code.
 */
int8_t bmv080_i2c_read(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length);

/**
 * @brief Implements the default I2C write transaction
 *
 * Sets the write function callback pointer on the bmv080 device struct.
 * Used throughout the Bosch library to write to the device. See the
 * typedef for bmv080_callback_write_t in bmv080_defs.h for additional details.
 *
 * @param[in] handle Handle for the I2C interface
 * @param[in] header Header information for the following _payload_.
 * @param[in] payload Payload to be written consisting of 16 bit words.
 * @param[in] payload_length Lumber of _payload_ elements to be written.
 * @return 0 if successful, otherwise the return value is an externally defined error code.
 */
int8_t bmv080_i2c_write(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length);

/**
 * @brief Implementation for a microsecond delay callback
 *
 * Used by the Bosch library to execute a software delay operation. See the
 * typedef for bmv080_callback_delay_t in bmv080_defs.h for additional details.
 *
 * @param[in] period_us Duration of the delay in microseconds
 */
int8_t bmv080_delay_cb(uint32_t period_us);

/**
 * @brief Validates the BMV080 sensor output for numeric correctness.
 *
 * Checks for NaN and out-of-bounds values in the PM2.5 reading. Returns false if any critical
 * value is invalid or the pointer is null. This function can be used to filter corrupt
 * or uninitialized frames before processing or conversion.
 *
 * @param[in] o Pointer to the BMV080 sensor output structure to validate
 * @return true if the output is valid, false otherwise
 */
bool bmv080_is_valid_output(const bmv080_output_t *o);

/**
 * @brief Converts BMV080 sensor output to a fixed-point representation.
 *
 * Maps float values from the BMV080 output structure into fixed-point fields suitable for
 * transmission or compact storage. PM values are scaled to 16-bit unsigned integers using
 * a linear transformation from 0–1000 µg/m³ to 0–65535. Runtime is converted to a 0.01-second
 * resolution. Obstruction and range validity flags are encoded in a compact bitfield.
 *
 * @param[in] o Pointer to the BMV080 sensor output structure to convert
 * @return bmv080_fixed_t structure containing the fixed-point values
 */
bmv080_fixed_t bmv080_to_fixed(const bmv080_output_t *o);

/**
 * @example main.c
 *
 * Example usage of the BMV080 sensor driver.
 * @note: Requires HAL peripheral drivers and initialization, specifically for i2c.
 * @note: Assumes an I2C_HandleTypeDef hi2c1 instance exists.
 *
 * @code
 * #include "bmv080_driver.h"
 *
 * int main(void) {
 *      bmv080_sensor_t bmv080;
 *      bmv080_init(&bmv080, &hi2c1);
 *      bmv080_configure_duty_cycle(&bmv080, 20); // 20 second cycle
 *      if (bmv080.status != E_BMV080_OK)
 *        printf("Failed to configure duty cycle: %d\r\n", bmv080.status);
 *      bmv080_start_duty_cycle(&bmv080);

 *      while (1)
 *      {
 *        if (bmv080_poll(&bmv080) == E_BMV080_OK && bmv080.data_available)
 *        {
 *          // Floating point output
 *          bmv080_print_output(&bmv080.output);

 *          // Convert to fixed-point representation
 *          bmv080_fixed_t fixed = bmv080_to_fixed(&bmv080.output);

 *          // Print fixed-point values (example)
 *          debug_print("Fixed output: runtime=%u (0.01s), PM1=%u, PM2.5=%u, PM10=%u, flags=0x%02X\r\n",
 *                            fixed.runtime_in_0_01_sec,
 *                            fixed.pm1,
 *                            fixed.pm2_5,
 *                            fixed.pm10,
 *                            fixed.flags);

 *           bmv080.data_available = false;
 *           debug_print("\r\n");
 *        }
 *        HAL_Delay(100); // Polling period
 *     }
 * }
 * @endcode
 */
