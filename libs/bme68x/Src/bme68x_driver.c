/*******************************************************************************
 * @file: bme68x_driver.c
 * @version: 0.1.0
 * @sources:
 *   - Bosch BME68x library and Arduino examples
 *     https://github.com/boschsensortec/Bosch-BME68x-Library
 ******************************************************************************/

#include "bme68x_driver.h"

void bme_init(bme68x_sensor_t *bme, I2C_HandleTypeDef *i2c_handle)
{
  if (bme == NULL)
  {
    return;
  }

  // Zero-initialize all members
  memset(bme, 0, sizeof(bme68x_sensor_t));

  // Assign the I2C handle
  bme->i2c_handle = i2c_handle;

  // Zero out the other struct members to ensure a clean initialization
  // memset(&bme->device, 0, sizeof(bme->device));
  // memset(&bme->conf, 0, sizeof(bme->conf));
  // memset(&bme->heatr_conf, 0, sizeof(bme->heatr_conf));
  // // Update this if we switch to an array of sensor_data items
  // // memset(bme->sensor_data, 0, sizeof(bme->sensor_data));
  // memset(&bme->sensor_data, 0, sizeof(bme->sensor_data));

  //
  // Set default values
  //
  // TODO:
  // Anything else from the begin method with i2c?
  // Set bme->device items:
  bme->device.intf_ptr = i2c_handle;
  // - TODO? variant ID
  bme->device.read = &bme_read;
  bme->device.write = &bme_write;
  // - TODO? delay_us delay function pointer
  bme->status = BME68X_OK;
  // bme->n_fields = 0;
  // bme->i_fields = 0;
  // Typical ambient temperature in Celsius
  bme->device.amb_temp = 25;
  // Assume sensor starts in sleep mode
  // bme->last_op_mode = BME68X_SLEEP_MODE;
}

int8_t bme_check_status(bme68x_sensor_t *bme) {
  if (bme->status < BME68X_OK)
  {
    return BME68X_ERROR;
  }
  else if (bme->status > BME68X_OK)
  {
    return BME68X_WARNING;
  }
  else
  {
    return BME68X_OK;
  }
}

void bme_set_TPH_default(bme68x_sensor_t *bme)
{
  bme_set_TPH(bme, BME68X_OS_2X, BME68X_OS_16X, BME68X_OS_1X);
}

void bme_set_TPH(bme68x_sensor_t *bme, uint8_t osTemp, uint8_t osPres, uint8_t osHum)
{
  bme->status = bme68x_get_conf(&bme->conf, &bme->device);

  if (bme->status == BME68X_OK)
  {
    bme->conf.os_hum = osHum;
    bme->conf.os_temp = osTemp;
    bme->conf.os_pres = osPres;

    bme->status = bme68x_set_conf(&bme->conf, &bme->device);
  }
}


int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
  I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)intf_ptr; // Cast it back to the correct type
  HAL_I2C_Master_Transmit(i2c_handle, BME68X_ADDR, &reg_addr, 1, HAL_MAX_DELAY);
  // TODO: Confirm casting away the const for reg-data will be ok, i.e. ensure the HAL
  // transmit will not modify this value
  HAL_I2C_Master_Transmit(i2c_handle, BME68X_ADDR, (uint8_t *)reg_data, 1, HAL_MAX_DELAY);
  return 0;
}

/**
 * @brief Function that implements the default I2C read transaction
 * @param reg_addr : Register address of the sensor
 * @param reg_data : Pointer to the data to write the sensor value to
 * @param length   : Length of the transfer
 * @param i2c_handle : Pointer to the stm32 I2C peripheral handle
 * @return 0 if successful, non-zero otherwise
 */

int8_t bme_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
  I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)intf_ptr; // Cast it back to the correct type
  HAL_I2C_Master_Transmit(i2c_handle, BME68X_ADDR, &reg_addr, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(i2c_handle, BME68X_ADDR, reg_data, 1, HAL_MAX_DELAY);
  return 0;
}

