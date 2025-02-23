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
  bme->device.intf = BME68X_I2C_INTF;
  // - TODO? variant ID
  bme->device.read = &bme_read;
  bme->device.write = &bme_write;
  bme->device.delay_us = bme_delay_us;
  // - TODO? delay_us delay function pointer
  bme->status = BME68X_OK;
  // bme->n_fields = 0;
  // bme->i_fields = 0;
  // Typical ambient temperature in Celsius
  bme->device.amb_temp = 25;
  // Assume sensor starts in sleep mode
  bme->last_op_mode = BME68X_SLEEP_MODE;
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

void bme_set_TPH(bme68x_sensor_t *bme, uint8_t os_temp, uint8_t os_pres, uint8_t os_hum)
{
  bme->status = bme68x_get_conf(&bme->conf, &bme->device);

  if (bme->status == BME68X_OK)
  {
    bme->conf.os_hum = os_hum;
    bme->conf.os_temp = os_temp;
    bme->conf.os_pres = os_pres;

    bme->status = bme68x_set_conf(&bme->conf, &bme->device);
    debug_print("in set_TPH after set_conf\r\n");
  }
}

void bme_set_heaterprof(bme68x_sensor_t *bme, uint16_t temp, uint16_t dur)
{
  bme->heatr_conf.enable = BME68X_ENABLE;
  bme->heatr_conf.heatr_temp = temp;
  bme->heatr_conf.heatr_dur = dur;

  bme->status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &bme->heatr_conf, &bme->device);
}

/**
 * @brief Function to fetch data from the sensor into the local buffer
 * @return Number of new data fields
 */
uint8_t bme_fetch_data(bme68x_sensor_t *bme) {
  // TODO: Update with struct members n_fields and i_fields
  // For now, hardcode
  // bme->n_fields = 0;
  debug_print("In bme_fetch_data with opmode: %d\r\n", bme->last_op_mode);
  uint8_t n_fields = 0;
  // bme->status = bme68x_get_data(bme->last_op_mode, bme->sensor_data, &bme->n_fields, &bme->device);
  bme->status = bme68x_get_data(bme->last_op_mode, &bme->sensor_data, &n_fields, &bme->device);
  // bme->i_fields = 0;

  // return bme->n_fields;
  return n_fields;
}

/**
 * @brief Function to get a single data field
 * @param data : Structure where the data is to be stored
 * @return Number of new fields remaining
 */
// uint8_t bme_get_data(bme68x_sensor_t *bme, bme68xData &data) {

// }

/**
 * @brief Function to get whole sensor data
 * @return Sensor data
 */
// bme68xData *bme_get_alldata(bme68x_sensor_t *bme) {

// }

/**
 * @brief Function to set the operation mode
 */
void bme_set_opmode(bme68x_sensor_t *bme, uint8_t opmode)
{
  bme->status = bme68x_set_op_mode(opmode, &bme->device);
  if ((bme->status == BME68X_OK) && (opmode != BME68X_SLEEP_MODE))
    bme->last_op_mode = opmode;
}

uint32_t bme_get_meas_dur(bme68x_sensor_t *bme, uint8_t opmode)
{
  // TODO: May need to fixup this "default" to BME68X_SLEEP_MODE
  // if (opmode == NULL) {
  //   opmode = BME68X_SLEEP_MODE;
  // }
  if (opmode == BME68X_SLEEP_MODE)
    opmode = bme->last_op_mode;

  return bme68x_get_meas_dur(opmode, &bme->conf, &bme->device);
}

// FIXME:
static TIM_HandleTypeDef htim6;

/**
 * @brief Function that implements the default microsecond delay callback
 */
void bme_delay_us(uint32_t period_us, void *intf_ptr)
{
  (void)intf_ptr;
  debug_print("Delay called for %lu us\r\n", period_us);
  // For microsecond delays less than 1ms, use a crude busy-wait
  if (period_us < 1000)
  {
    volatile uint32_t cycles = period_us * 20; // Adjust based on your clock speed
    while (cycles--)
    {
      __NOP();
    }
  }
  else
  {
    // For longer delays, use HAL_Delay
    HAL_Delay((period_us + 999) / 1000); // Round up to nearest ms
  }
  debug_print("Delay completed\r\n");
  // TODO? May need to replace this HAL_Delay
  // HAL_Delay(period_us);
  // DWT_Delay_us(period_us);
  // HAL_Delay(1);
  // static uint8_t initialized = 0;

  // Initialize timer on first call
  // Timer should be initialized with MX_TIM6_Init() in the main function
  // if (!initialized)
  // {
  //   timer_init();
  //   initialized = 1;
  // }

  /*
  // Handle delays longer than the timer's period
  while (period_us > 0xFFFF)
  {
    // Start timer
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start(&htim6);

    // Wait for maximum period
    while (__HAL_TIM_GET_COUNTER(&htim6) < 0xFFFF)
    {
    }

    HAL_TIM_Base_Stop(&htim6);
    period_us -= 0xFFFF;
  }

  // Handle remaining time
  if (period_us > 0)
  {
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    HAL_TIM_Base_Start(&htim6);

    while (__HAL_TIM_GET_COUNTER(&htim6) < period_us)
    {
    }

    HAL_TIM_Base_Stop(&htim6);
  }
  */
}

int8_t bme_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
  I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)intf_ptr; // Cast it back to the correct type

  // Create a buffer to store the register address followed by the data
  uint8_t buffer[length + 1];
  buffer[0] = reg_addr;                 // First byte is the register address
  memcpy(&buffer[1], reg_data, length); // Copy the data after the register address

  // Transmit register address + data
  if (HAL_I2C_Master_Transmit(i2c_handle, BME68X_ADDR, buffer, length + 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1; // Error in transmission
  }

  return 0; // Success
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

  // Send the register address (1 byte)
  if (HAL_I2C_Master_Transmit(i2c_handle, BME68X_ADDR, &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1; // Error in transmitting register address
  }

  // Read the requested number of bytes
  if (HAL_I2C_Master_Receive(i2c_handle, BME68X_ADDR, reg_data, length, HAL_MAX_DELAY) != HAL_OK)
  {
    return -1; // Error in reading data
  }

  return 0; // Success
}

// FIXME:
static uint32_t cpu_freq_hz = 0;
static uint32_t cycles_per_us = 0;

// Enable DWT and start the cycle counter
void DWT_Delay_Init(void)
{
  /*
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // Enable DWT
  DWT->CYCCNT = 0;                                // Reset cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // Enable cycle counter
  */
  // Enable trace and debug clocks
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

  // Reset DWT control register and cycle counter
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  // Enable cycle counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // Get CPU frequency in Hz
  cpu_freq_hz = HAL_RCC_GetHCLKFreq();
  cycles_per_us = cpu_freq_hz / 1000000U;
}

// Microsecond delay using DWT cycle counter
void DWT_Delay_us(uint32_t us)
{
  /*
  uint32_t clk_cycle_start = DWT->CYCCNT;
  uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000) * us; // Convert us to clock cycles
  while ((DWT->CYCCNT - clk_cycle_start) < cycles)
    ; // Wait until enough cycles have passed
    */
  // Initialize DWT on first call
  if (cycles_per_us == 0)
  {
    DWT_Delay_Init();
  }

  uint32_t start_cycles = DWT->CYCCNT;
  uint32_t target_cycles = us * cycles_per_us;

  // Wait until enough cycles have elapsed
  while ((DWT->CYCCNT - start_cycles) < target_cycles)
  {
    __NOP(); // Prevent optimizer from removing the loop
  }
}
