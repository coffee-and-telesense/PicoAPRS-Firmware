# Bosch BME68x Driver

## Overview

This driver is used to initialize and configure a Bosch 68x sensor (specifically the [688](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme688/) or [680](https://www.bosch-sensortec.com/products/environmental-sensors/gas-sensors/bme680/)), and retrieve data for temperature, pressure, humidity, and a resistance value reflecting gas composition. Communication with the sensor takes place over I2C, using an I2C handle created with the STM32 HAL.

Driver functionality is focused on low-power environments with minimal code storage. There may be features supported by the 688, for example, that are not supported here because they have requirements that are outside the anticipated operating context for the Pico Balloon platform.

### Driver Structure

The driver is based on a combination of a library provided by Bosch with a simplified interface using the STM32 HAL. The files containing code from Bosch include `bme68x_def.h`, `bme68x.h`, and `bme68x.c`. The STM32 HAL support and simplified interface is contained in `bme68x_driver.h` and `bme68x_driver.c`.

The Bosch code was retrieved from the [Bosch-BME68x-Library github repo](https://github.com/boschsensortec/Bosch-BME68x-Library/tree/master) and checked into this repository in whole. The library was then reduced to support only the necessary functionality. See the [Design Decisions](#design-decisions) below for additional detail.

In addition, the Bosch library github repo contained an Arduino library written in C++. This served as a model for many of the functions provided in this driver, with appropriate modifications made for the STM32.

### Example Usage

The following example shows creation and initialization of a `bme68x_sensor_t` struct, which contains a pointer to an I2C handle previously initialized in application code. The struct also contains status, a BME68x device struct, configuration, sensor data, and the last operation mode.

After initialization, configuration is performed, using default oversampling values for the temperature, pressure, and humidity measurements, and a heater configuration of 300 deg C for 100ms in forced mode.

Data is then fetched from the sensor repeatedly in a loop. The specific delay function may be adjusted to use a hardware timer, but the microsecond delay should still be based on the return value from the `bme_get_meas_dur` function.

```c
#include "bme68x_driver.h"

int main(void) {
  bme68x_sensor_t bme;
  bme_init(&bme, &hi2c1);
  bme_set_TPH_default(&bme);
  bme_set_heaterprof(&bme, 300, 100);

  while (1)
  {
    bme_set_opmode(&bme, BME68X_FORCED_MODE);
    /** @todo: May adjust the specific timing function called here, but it should be based on bme_get_meas_dur */
    bme_delay_us(bme_get_meas_dur(&bme, BME68X_SLEEP_MODE), &hi2c1);
    int fetch_success = bme_fetch_data(&bme);
    if (fetch_success) {
          debug_print("%d, ", bme.sensor_data.temperature);
          debug_print("%d, ", bme.sensor_data.pressure);
          debug_print("%d, ", bme.sensor_data.humidity);
          debug_print("%d, ", bme.sensor_data.gas_resistance);
          debug_print("%X, \r\n", bme.sensor_data.status);
        }
  }
}

```

## Driver API

### `bme68x_sensor_t` struct

Structure to store necessary configuration and data to interact with the sensor over I2C. Contains as members the structs `bme68x_dev`, `bme68x_conf`, `bme68x_heatr_conf`, and `bme68x_data`, which are defined in `bme68x_defs.h`. Also contains an `I2C_HandleTypeDef` pointer to allow I2C communication using STM32 HAL I2C functions.

### `bme_init()`

Configures the sensor with default settings.

**Parameters:**
- `bme`: Pointer to newly initialized bme68x sensor interface
- `i2c_handle`: Pointer to I2C handle

### `bme_check_status()`

Checks and returns current bme instance status. Returns 0 for status OK, 1 for warning, -1 for error. **NOTE**: This may be modified in the future to leverage the common error type construction defined in [libs/common/Inc/error_types.h](../common/Inc/error_types.h).

**Parameters:**
- `bme`: Pointer to bme68x sensor interface

**Returns:**
- `int8_t`: 0 for status 0, 1 for warning, -1 for error

### `bme_set_TPH_default()`

Set the Temperature, Pressure and Humidity over-sampling using the following defaults: 2 samples for temperature, 16 for pressure, 1 for humidity. These defaults are based on those used in the self-test check function provided in the Bosch library.

**Parameters:**
- `bme`: Pointer to bme68x sensor interface

### `bme_set_TPH()`

Set the Temperature, Pressure and Humidity over-sampling.

**Parameters:**
- `bme`: Pointer to bme68x sensor interface
- `os_temp`: Number of temperature measurements to perform, BME68X_OS_NONE to BME68X_OS_16X
- `os_pres`: Number of pressure measurements to perform, BME68X_OS_NONE to BME68X_OS_16X
- `os_hum`: Number of humidity measurements to perform, BME68X_OS_NONE to BME68X_OS_16X

### `bme_set_heaterprof()`

Set the heater profile for Forced mode. Parameters for temperature and duration may be adjusted depending on the gas profile being targeted.

**Parameters:**
- `bme`: Pointer to bme68x sensor interface
- `temp`: Heater temperature in degree Celsius
- `dur`: Heating duration in milliseconds

### `bme_set_opmode()`

Set the operation mode. Currently supports sleep mode and forced mode for a single, low-power measurment which may automatically return to sleep mode. The gas sensor heater only operates during measurement in this mode.

**Parameters:**
- `bme`: Pointer to bme68x sensor interface
- `opmode`: BME68X_SLEEP_MODE or BME68X_FORCED_MODE

### `bme_fetch_data()`

Fetch data from the sensor into the struct's sensor_data buffer. **NOTE**: This should typically return 1, indicating new data was retrieved. This value could be more than 1 when implementing support for parallel or sequential data retrieval modes.

**Parameters:**
- `bme`: Pointer to bme68x sensor interface

**Returns:**
- `uint8_t`: Number of new data fields

### `bme_get_meas_dur()`

Get the measurement duration in microseconds. This is the total measurement duration based on the total number of samples to be taken, as determined by the oversampling for each value.

**Parameters:**
- `bme`: Pointer to bme68x sensor interface
- `opmode`: Operation mode of the sensor. Attempts to use the last one if nothing is set

**Returns:**
- `uint32_t`: Measurement duration in microseconds

### `bme_delay_us()`

Implements the default microsecond delay callback, as called from the Bosch library. **NOTE**: This function signature is currently determined by the original one used by the Bosch library developers. We may wish to modify it in the future to better accommodate our usage of the STM32 HAL timing functions.

**Parameters:**
- `period_us`: Duration of the delay in microseconds
- `intf_ptr`: Pointer to the interface descriptor for I2C

### `bme_write()`

Implements the default I2C write transaction. This is leveraged by the Bosch library to perform writes to the device over I2C. It should typically not be necessary for application code to call this function directly, though it may be useful in troubleshooting.

**Parameters:**
- `reg_addr`: Register address of the sensor
- `reg_data`: Pointer to the data to be written to the sensor
- `length`: Length of the transfer
- `i2c_handle`: Pointer to the stm32 I2C peripheral handle

**Returns:**
- `int8_t`: 0 if successful, non-zero otherwise

### `bme_read()`

Implements the default I2C read transaction. This is leveraged by the Bosch library to perform reads from the device over I2C. It should typically not be necessary for application code to call this function directly, though it may be useful in troubleshooting.

**Parameters:**
- `reg_addr`: Register address of the sensor
- `reg_data`: Pointer to the data to write the sensor value(s) to
- `length`: Length of the transfer
- `i2c_handle`: Pointer to the stm32 I2C peripheral handle

**Returns:**
- `int8_t`: 0 if successful, non-zero otherwise

## Design Decisions

As noted above in the overview, some functionality has been removed from the Bosch-provided library. This was based on expected operating conditions, considering factors such as power usage and flash storage. Below is an outline of functionality that was removed from the Bosch library, as well as functionality that was left in place, with brief explanations for each.

The entirety of the Bosch library has been checked in to our codebase, so retrieving and re-implementing removed features in the future should be possible if desired.

### Removed functionality

**Parallel and sequential modes**

Parallel mode is not supported by the BME680, and in the case of the 688, it is described as requiring more power than forced mode. Parallel mode also does not support automatically returning to sleep mode. In addition, the greatest benefit from parallel mode is likely achieved by using the AI training features possible with the 688. These features require usage of a static library provied by Bosch. At the current time, it is assumed that we should prioritize a smaller library size over utilizing this feature.

Sequential mode is not documented in the 688 datasheet, but it seems likely this is also intended to be leveraged along with the AI training on targeting specific gas profiles. It allows for including multiple temperature and duration settings to be used for sequential measurements.

**SPI support**

We do not currently intend to support the SPI protocol for communication with sensors.

### Maintained functionality

**Conditional compilation for a kernel context**

The `bme68x_defs.h` file provided by Bosch includes some conditional compilation sections (e.g. `#ifdef __KERNEL__`) which seem to be aimed at supporting usage of the library within a Linux kernel context. These have been left in place to allow for the possibility of RTOS usage in the future.

**Conditional compilation for hardware floating point support**

It is not necessarily anticipated that we will use floating point hardware, but the Bosch driver includes support for it. As with the above conditional compilation for a kernel context, this code does not contribute to the overall driver size as long as compilation correctly includes the `BME68X_DO_NOT_USE_FPU` definition. This is currently added in the `CMakeLists.txt` file for the driver.

**Currently unused functions**

There are a few functions from the Bosch library that are not currently in use, but could possibly be useful in the future (e.g. `sort_sensor_data`, `calc_heatr_dur_shared`, and `read_all_field_data`). These have been left in place under the assumption that the application will be compiled with some reasonable level of optimization which will result in their removal from the compiled binary.

## Next Steps

**Update delay function**

The current delay function was added to allow for retrieval of data in a prototyping context. It should be replaced, probably with a hardware timer. Usage of the `HAL_Delay` function was attempted, but that function provides millisecond delays, rather that the microsecond delays required by this driver.

Other alternatives might be using the DWT as a cycle counter, or modifying the Bosch library callback signature to be more specific to our needs.

**Validate humidity measurements**

The humidity measurements currently appear to be returning the max value consistently. This may be the result of the above prototyping-level delay function, or possibly an individual sensor-specific problem. First steps to resolve are to update the above delay function, test on additional sensors, and adjust the oversampling setting.

**Create non-blocking version**

We should allow for the driver to be used in a non-blocking manner, leveraging interrupts.

**Ensure compatibility with an RTOS**

This work is TBD, but the maintenance of the conditional compilation for the Linux kernel supports this work.

**Use standardized error types**

We should incorporate the common platform error types (see [libs/common/Inc/error_types.h](../common/Inc/error_types.h)). Specific functions where the standardized error types may be helpful include `bme_write`, `bme_read`, and `bme_check_status`.
