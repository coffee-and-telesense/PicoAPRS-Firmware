/*******************************************************************************
 * @file: bmv080_driver.c
 * @brief: Driver for the BMV080 sensor using STM32 HAL.
 *
 * This file contains the implementation of functions for configuring
 * and reading data from the BMV080 sensor using STM32 HAL.
 *
 * @version: 0.1.0
 ******************************************************************************/

#include "bmv080_driver.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

/** @brief Implements the default I2C read transaction */
int8_t i2c_read_16bit_cb(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length) {
  HAL_StatusTypeDef return_value;
  I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)handle;

  /* verify function parameters */
  if ((handle == NULL) || (payload == NULL))
  {
    return HAL_ERROR;
  }

  if ((payload_length == 0))
  {
    return HAL_ERROR;
  }

  /* 7 bit I2C address + most significant bit of the header passed from the sensor driver which represents the R/W bit (R = high) */
  uint16_t device_address = (BMV080_I2C_ADDRESS << 1) | (header >> 15);
  /* 16 bit header left shifted, since the R/W bit has already been passed */
  uint16_t header_adjusted = header << 1;

  return_value = HAL_I2C_Mem_Read(i2c_handle, device_address, header_adjusted, I2C_MEMADD_SIZE_16BIT, (uint8_t *)payload, payload_length * 2, HAL_MAX_DELAY);

  /* Conversion of payload from big endian to little endian */
  for (int payload_idx = 0; payload_idx < payload_length; payload_idx++)
  {
    uint16_t swapped_word = ((payload[payload_idx] << 8) | (payload[payload_idx] >> 8)) & 0xffff;
    payload[payload_idx] = swapped_word;
  }

  return (int8_t)return_value;
}

/** Implements the default I2C write transaction */
int8_t i2c_write_16bit_cb(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length) {
  HAL_StatusTypeDef return_value;
  I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)handle;

  /* verify function parameters */
  if ((handle == NULL) || (payload == NULL))
  {
    return HAL_ERROR;
  }

  if ((payload_length == 0))
  {
    return HAL_ERROR;
  }

  /* 7 bit I2C address + most significant bit of the header passed from the sensor driver which represents the R/W bit (R = high) */
  uint16_t device_address = (BMV080_I2C_ADDRESS << 1) | (header >> 15);
  /* 16 bit header left shifted, since the R/W bit has already been passed */
  uint16_t header_adjusted = header << 1;

  /* Conversion of payload from little endian to big endian (dynamic allocation is used) */
  uint16_t *payload_swapped = (uint16_t *)calloc(payload_length, sizeof(uint16_t));
  for (int payload_idx = 0; payload_idx < payload_length; payload_idx++)
  {
    payload_swapped[payload_idx] = ((payload[payload_idx] << 8) | (payload[payload_idx] >> 8)) & 0xffff;
  }

  return_value = HAL_I2C_Mem_Write(i2c_handle, device_address, header_adjusted, I2C_MEMADD_SIZE_16BIT, (uint8_t *)payload_swapped, payload_length * 2, HAL_MAX_DELAY);

  free(payload_swapped);

  return (int8_t)return_value;
}

/** Implementation for a microsecond delay callback */
int8_t delay_cb(uint32_t period_us) {
  HAL_Delay(period_us);
  return HAL_OK;
}

/** Validates the BMV080 sensor output for numeric correctness. */
bool bmv080_is_valid_output(const bmv080_output_t *o)
{
  if (!o)
    return false;

  return !isnan(o->runtime_in_sec) &&
         !isnan(o->pm2_5_mass_concentration) &&
         o->pm2_5_mass_concentration >= 0.0f &&
         o->pm2_5_mass_concentration <= 1000.0f;
}

/** Converts BMV080 sensor output to a fixed-point representation. */
bmv080_fixed_t bmv080_to_fixed(const bmv080_output_t *o)
{
  bmv080_fixed_t out = {0};

  if (!o) {
    return out;
  }

  // Clamp and scale PM values: float [0–1000] -> uint16_t [0–65535]
  float scale = 65535.0f / 1000.0f;

  out.pm1 = (o->pm1_mass_concentration < 0.0f) ? 0 : (o->pm1_mass_concentration > 1000.0f) ? 65535
                                                                                           : (uint16_t)(o->pm1_mass_concentration * scale + 0.5f);

  out.pm2_5 = (o->pm2_5_mass_concentration < 0.0f) ? 0 : (o->pm2_5_mass_concentration > 1000.0f) ? 65535
                                                                                                 : (uint16_t)(o->pm2_5_mass_concentration * scale + 0.5f);

  out.pm10 = (o->pm10_mass_concentration < 0.0f) ? 0 : (o->pm10_mass_concentration > 1000.0f) ? 65535
                                                                                              : (uint16_t)(o->pm10_mass_concentration * scale + 0.5f);

  // Runtime in 0.01-second resolution, clipped at ~655s
  out.runtime_in_0_01_sec = (uint16_t)(fminf(fmaxf(o->runtime_in_sec, 0.0f), 655.35f) * 100.0f + 0.5f);

  // Set flags
  out.flags = 0;
  if (o->is_obstructed)
    out.flags |= 0x01;
  if (o->is_outside_measurement_range)
    out.flags |= 0x02;

  return out;
}
