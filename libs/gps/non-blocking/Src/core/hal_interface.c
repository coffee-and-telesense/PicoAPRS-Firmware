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

#include "hal_interface.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"

/**
 * @brief Global interface instance
 *
 * This pointer stores the active interface instance, allowing HAL callbacks
 * to route events to the correct driver callbacks. A more complex implementation
 * could support multiple instances using an array or linked list.
 *
 * Example:
 *  ---> static hal_interface_t g_hal_interfaces[MAX_INTERFACES];
 */
static hal_interface_t* g_hal_interface = NULL;

/**
  * @brief Non-blocking delay implementation using timer
  *
  * Configures and starts a timer for the requested delay duration.
  * When the timer expires, it triggers the delay_complete callback.
  * TODO: Delay duration is not implemented yet in this version, instead
  * the timer is started and the delay_complete callback is triggered when the timer expires, which
  * has currently been configured with CubeMX.
  */
static HAL_StatusTypeDef hal_delay_it(TIM_HandleTypeDef *htim, uint32_t delay_ms);

 /**
  * @brief Initialize the HAL interface
  * This simplified version manages a single interface instance, making it
  * easier to track state and handle callbacks. We store the interface in a
  * global variable so HAL callbacks can access it.
  *
  * @param hal Interface instance to initialize
  * @return HAL_OK if successful, HAL_ERROR otherwise
  */
HAL_StatusTypeDef hal_interface_init(hal_interface_t *hal) {
    // Basic parameter validation
    if(!hal || (!hal->hi2c && !hal->htim)) {
        #ifdef DEBUG
            debug_print("Invalid parameters\n");
        #endif
         return HAL_ERROR;
    }

    // Store our single interface instance
    g_hal_interface = hal;

    // Initialize callbacks to safe default state - these should be set by the driver
    // after initializing the HAL interface.
    hal->callbacks.context = NULL;
    hal->callbacks.tx_complete = NULL;
    hal->callbacks.rx_complete = NULL;
    hal->callbacks.error = NULL;
    hal->callbacks.delay_complete = NULL;

    // Wire up the HAL function pointers - these map directly to STM32 HAL functions
    // TODO: This interface should only wire up the functions that are actually used
    // by the driver, rather than all of them. For now wire up everything.
    hal->ops.transmit_blocking = HAL_I2C_Master_Transmit;
    hal->ops.receive_blocking = HAL_I2C_Master_Receive;
    hal->ops.transmit_it = HAL_I2C_Master_Transmit_IT;
    hal->ops.receive_it = HAL_I2C_Master_Receive_IT;
    hal->ops.delay_it = hal_delay_it;

    return HAL_OK;
 }

 /**
  * @brief Non-blocking delay implementation using timer
  *
  * Configures and starts a timer for the requested delay duration.
  * When the timer expires, it triggers the delay_complete callback.
  */
HAL_StatusTypeDef hal_delay_it(TIM_HandleTypeDef *htim, uint32_t delay_ms) {
    (void)delay_ms;  // Unused parameter for now
    if(!htim) return HAL_ERROR;

    // Configure timer for millisecond-precision delays
    /** FIXME: These configs were done in cubemx, but maybe they could be done here to
     * make the code more portable
     uint32_t timer_freq = HAL_RCC_GetPCLK1Freq();
     uint32_t prescaler = (timer_freq / 1000) - 1;   // Set up millisecond ticks
     uint32_t period = delay_ms - 1;                 // Timer counts from 0

     htim->Instance->PSC = prescaler;
     htim->Instance->ARR = period;
     htim->Instance->CNT = 0;  // Reset counter
    */
    return HAL_TIM_Base_Start_IT(htim);
 }

 /**
  * @brief Clean up the HAL interface
  *
  * Simple cleanup that clears our global reference. In a more complex system,
  * we might need to handle ongoing transactions or disable interrupts.
  */
 void hal_interface_deinit(hal_interface_t *hal) {
     if(hal && (hal == g_hal_interface)) {
         g_hal_interface = NULL;
     }
 }

 /*
  * HAL Callback Implementations
  *
  * These functions are called by the STM32 HAL when operations complete. The magic of this
  * is that these callbacks are registered with the HAL interface, so the driver doesn't need to
  * know about them. The driver just needs to implement,and set the appropriate callbacks for its own use.
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;  // Unused parameter for now since we only have one interface
    #ifdef DEBUG
        debug_print("HAL_I2C_MasterTxCpltCallback Tx complete\n");
    #endif
    if(g_hal_interface && g_hal_interface->callbacks.tx_complete) { // Check if the callback is set
        g_hal_interface->callbacks.tx_complete(g_hal_interface->callbacks.context);
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;  // Unused parameter for now since we only have one interface
    #ifdef DEBUG
        debug_print("HAL_I2C_MasterRxCpltCallback Rx complete\n");
    #endif
    if(g_hal_interface && g_hal_interface->callbacks.rx_complete) {
        g_hal_interface->callbacks.rx_complete(g_hal_interface->callbacks.context);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;  // Unused parameter for now since we only have one interface
    #ifdef DEBUG
        debug_print("HAL_I2C_ErrorCallback\n");
    #endif
    if(g_hal_interface && g_hal_interface->callbacks.error) {
        g_hal_interface->callbacks.error(g_hal_interface->callbacks.context, HAL_ERROR);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    #ifdef DEBUG
        debug_print("HAL_TIM_PeriodElapsedCallback Timer Complete\n");
    #endif
    if(g_hal_interface && g_hal_interface->callbacks.delay_complete) {
        HAL_TIM_Base_Stop_IT(htim);  // Stop the timer
        g_hal_interface->callbacks.delay_complete(g_hal_interface->callbacks.context);
    }
    else{
        #ifdef DEBUG
            debug_print("Delay complete callback not set\n");
        #endif
    }
}
