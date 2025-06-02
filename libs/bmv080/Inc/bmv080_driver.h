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

// TODO: Include example usage
