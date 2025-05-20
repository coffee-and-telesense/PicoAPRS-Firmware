#pragma once
/**
 * @file max_m10s.h
 * @brief Driver for the MAX-M10 GPS module intended for use as a GPS beacon.
 *        Configurations and API calls are minimized to limit the scope of the driver
 *        to the basic functionality required for the Pico Balloon project. Driver can
 *        be ran in both  blocking and non-blocking modes depending on the configuration.
 *        Non-blocking mode is enabled by defining the NON_BLOCKING macro.
 *
 * @note In non-blocking mode the application needs to consider special care for synchronization
 *      and timing of the I2C operations. The driver will not handle any synchronization. In blocking
 *      mode, the application only needs to consider the time delay between writing to the GPS
 *      and reading from it.
 *
 * @todo
 *      - Consider adding HAL abstraction layer to prevent external dependencies.
 *      - Power saving features need to be added. See pg. 36 of integration manual.
 */
#include <stdint.h>
#include "gps_types.h"
#include "ubx.h"
#include "i2c.h"

//=============================================================================
// Type definitions
//=============================================================================

// Function pointer for blocking delay
typedef void (*delay_blocking_fn)(uint32_t delay);

/* Function pointer typedefs using direct STM32 HAL types */
#ifdef NON_BLOCKING
    typedef HAL_StatusTypeDef (*i2c_transmit)(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                              uint8_t *data, uint16_t size);

    typedef HAL_StatusTypeDef (*i2c_receive)(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                             uint8_t *data, uint16_t size);
#else
    typedef HAL_StatusTypeDef (*i2c_transmit)(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                              uint8_t *data, uint16_t size, uint32_t timeout);

    typedef HAL_StatusTypeDef (*i2c_receive)(I2C_HandleTypeDef* hi2c, uint16_t addr,
                                             uint8_t *data, uint16_t size, uint32_t timeout);
#endif

/**
 * @brief Configuration structure for the MAX-M10S GPS driver.
 *
 * @example Initialization Example:
 *   max_m10s_init_s gps_init = {
 *     .hi2c = &hi2c1,
 *     .device_address = 0x42,
 *     .timeout_ms = 1000,
 *     .delay_blocking = HAL_Delay,  // if using rtos use OS Delay metho instead
 *     #ifdef NON_BLOCKING
 *       .transmit = HAL_I2C_Master_Transmit_IT,
 *       .receive = HAL_I2C_Master_Receive_IT,
 *     #else
 *       .transmit = HAL_I2C_Master_Transmit,
 *       .receive = HAL_I2C_Master_Receive,
 *     #endif
 *   };
 */
typedef struct {
    I2C_HandleTypeDef* hi2c;
    /* Function Pointers */
    i2c_transmit transmit;
    i2c_receive receive;
    delay_blocking_fn delay_blocking;
    /* Basic Configurations */
    uint8_t device_address;
    uint32_t timeout_ms;
} max_m10s_init_s;

/**
 * @brief Device structure for MAX-M10S GPS module
 */
typedef struct {
    uint8_t initialized;
    gps_cmd_type_e current_cmd;     // Current command being executed
    max_m10s_init_s configs;
    uint8_t tx_buffer[MAX_BUFFER_SIZE];         // TODO: Determine actual size needed
    uint8_t rx_buffer[MAX_BUFFER_SIZE];         // TODO: Determine actual size needed
    uint16_t tx_size;
    uint16_t rx_size;
} max_m10s_dev_s;

//=============================================================================
// Macros for internal use
//=============================================================================

#ifdef NON_BLOCKING
    /**
     * @brief Inserts wait states for I2C to complete in non-blocking mode
     * @param dev Device pointer
     * @return Status code
     */
    gps_status_e i2c_wait_for_complete(max_m10s_dev_s *dev);
    #define I2C_WAIT(dev) i2c_wait_for_complete(dev)

    // Macro to handle the transmit call with proper arguments
    #define GPS_TRANSMIT(dev, addr, data, size) \
        dev->configs.transmit(dev->configs.hi2c, addr, data, size)

    // Macro to handle the receive call with proper arguments
    #define GPS_RECEIVE(dev, addr, data, size) \
        dev->configs.receive(dev->configs.hi2c, addr, data, size)
#else
    #define I2C_WAIT(dev) /* empty */

    // Macro to handle the transmit call with timeout
    #define GPS_TRANSMIT(dev, addr, data, size) \
        dev->configs.transmit(dev->configs.hi2c, addr, data, size, dev->configs.timeout_ms)

    // Macro to handle the receive call with timeout
    #define GPS_RECEIVE(dev, addr, data, size) \
        dev->configs.receive(dev->configs.hi2c, addr, data, size, dev->configs.timeout_ms)
#endif

//=============================================================================
// Public API Functions
//=============================================================================

/**
 * @brief Initialize the MAX-M10S GPS driver with minimal configuration
 *          - Sets UBX protocol to I2C Output
 *          - Turns of NMEA output
 * @param dev Pointer to device structure to initialize
 * @param init Pointer to initialization configuration
 * @return Status code (UBLOX_OK on success)
 */
gps_status_e max_m10s_init(max_m10s_dev_s *dev, const max_m10s_init_s *init);

/**
 * @brief Send a command to the GPS device
 * @param dev Pointer to MAX-M10S device structure
 * @param cmd_type Type of command to execute (from enum gps_cmd_type_e)
 * @return UBLOX_OK if request initiated successfully; else error code
 */
gps_status_e max_m10s_command(max_m10s_dev_s *dev, gps_cmd_type_e cmd_type);

/**
 * @brief Read data from the GPS device
 * @param dev: Pointer to MAX-M10S device structure
 * @return UBLOX_OK if read initiated successfully; else error code
 */
gps_status_e max_m10s_read(max_m10s_dev_s *dev);

/**
 * @brief Validate a received packet from the GPS device
 * @param dev: Pointer to MAX-M10S device structure
 * @param cmd_type: Type of command for which the response should be validated
 * @return UBLOX_OK if packet is valid; else error code
 */
gps_status_e max_m10s_validate_response(max_m10s_dev_s *dev, gps_cmd_type_e cmd_type);


/**
 * @brief Configs the measurement rate of the MAX-M10S GPS device
 *
 * @param dev: Pointer to MAX-M10S device structure
 * @param rate: Measurement rate in ms (1000ms = 1Hz), (100ms = 10Hz)
 * @return gps_status_e
 */
gps_status_e max_m10s_config_meas_rate(max_m10s_dev_s *dev, uint16_t rate);
