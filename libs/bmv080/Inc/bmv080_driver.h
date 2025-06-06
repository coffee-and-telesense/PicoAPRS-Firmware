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

/* ========================== MACROS ========================== */
/** @note: The datasheet states that the default I2C address is 0x57, but the Bosch
 * example code uses 0x54 as the default. User should confirm the address and override
 * as needed.
 */
#define BMV080_I2C_ADDRESS 0x57

/* ========================== TYPE DEFINITIONS ========================== */

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
int8_t i2c_read_16bit_cb(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length);

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
int8_t i2c_write_16bit_cb(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length);

/**
 * @brief Implementation for a microsecond delay callback
 *
 * Used by the Bosch library to execute a software delay operation. See the
 * typedef for bmv080_callback_delay_t in bmv080_defs.h for additional details.
 *
 * @param[in] period_us Duration of the delay in microseconds
 */
int8_t delay_cb(uint32_t period_us);

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

// TODO: Include example usage
