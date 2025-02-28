/**
 * @file hal_interface.h
 * @brief Hardware Abstraction Layer (HAL) interface for gps module
 *
 * @details This interface provides a hardware-independent layer between device drivers
 *          and microcontroller-specific code. It primarily supports I2C communication
 *          and timer-based delays.
 *
 * This layer allows flexibility to how I've written the driver code, as it can be use function pointers
 * to register callbacks defined by the driver. This allows the driver to be non-blocking and
 * to handle errors and completion events in a more flexible way.
 *
 * Key features:
 * - Non-blocking I2C operations with completion callbacks
 * - Timer-based delay operations
 *
 * @sources:
 *      - https://medium.com/@kasra_mp/implementing-callback-functions-using-function-pointers-in-c-3e12838ef222
 *      - https://medium.com/@khimaja3127/how-to-design-interfaces-in-c-that-work-for-you-e3f509b188dd
 *
 *
 * @note This implementation currently supports a single interface instance through
 *       a global variable. It could be extended to support multiple instances if needed.
 */



#pragma once
#include "stm32l4xx_hal.h"
#include "logging.h"

// Forward declarations for generic HAL callbacks
// These are used by the HAL interface to notify driver of hardware events
typedef void (*hal_tx_complete_cb)(void* context);
typedef void (*hal_rx_complete_cb)(void* context);
typedef void (*hal_error_cb)(void* context, HAL_StatusTypeDef error);
typedef void (*hal_delay_complete_cb)(void* context);


/**
 * @brief Structure containing all callback functions and context
 *
 * This structure allows drivers to register callbacks for various hardware events.
 * The context pointer lets callbacks know which device instance they're servicing. In
 * this implementation it will always just be the one device instance.
 */
typedef struct {
    void* context;                      /**< Driver context pointer */
    hal_tx_complete_cb tx_complete;     /**< Transmission complete callback */
    hal_rx_complete_cb rx_complete;     /**< Receive complete callback */
    hal_error_cb error;                 /**< Error handling callback */
    hal_delay_complete_cb delay_complete; /**< Delay complete callback */
} hal_callbacks_t;


/**
 * @brief Main HAL interface structure
 *
 * Contains hardware handles, callbacks, and operation function pointers.
 * Driver will interact with hardware exclusively through this interface.
 */
typedef struct {
    I2C_HandleTypeDef *hi2c;           /**< I2C hardware handle */
    TIM_HandleTypeDef *htim;           /**< Timer hardware handle */
    hal_callbacks_t callbacks;          /**< Callback functions */
    struct {
        // I2C operations
        HAL_StatusTypeDef (*transmit_blocking)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                             uint8_t *data, uint16_t size, uint32_t timeout);
        HAL_StatusTypeDef (*receive_blocking)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                            uint8_t *data, uint16_t size, uint32_t timeout);
        HAL_StatusTypeDef (*transmit_it)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                        uint8_t *data, uint16_t size);
        HAL_StatusTypeDef (*receive_it)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                       uint8_t *data, uint16_t size);
        // Delay operations
        HAL_StatusTypeDef (*delay_it)(TIM_HandleTypeDef *htim, uint32_t delay_ms);
    } ops;  /**< Hardware operations */
} hal_interface_t;

/**
 * @brief Initialize the HAL interface
 * @param hal Pointer to interface structure to initialize
 * @return HAL_OK if successful, HAL_ERROR if invalid parameters
 */
HAL_StatusTypeDef hal_interface_init(hal_interface_t *hal);
