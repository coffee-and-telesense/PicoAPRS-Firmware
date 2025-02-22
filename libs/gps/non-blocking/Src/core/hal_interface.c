/**
 * @file hal_interface.c
 * @brief Simplified HAL interface implementation for single I2C and Timer instance
 */

#include "hal_interface.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"
#include "stm32l4xx_hal_tim.h"

// Single global interface instance - much simpler than tracking multiple instances but could be expanded
// to support multiple interfaces if needed.
// Example:
// -> static hal_interface_t g_hal_interfaces[MAX_INTERFACES];
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
  *
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
    // by the driver, rather than all of them. For now we'll just wire up everything.
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
  * These functions are called by the STM32 HAL when operations complete.
  * Since we only have one interface, we can directly use our global variable
  * instead of searching through an array of instances.
  */

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;  // Unused parameter for now since we only have one interface
    #ifdef DEBUG
        debug_print("HAL_I2C_MasterTxCpltCallback Tx complete\n");
    #endif
    if(g_hal_interface && g_hal_interface->callbacks.tx_complete) {
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

 /**
  * @brief Check if the current transaction has timed out
  *
  * A simple utility function that compares the elapsed time against
  * the transaction's timeout value.

bool hal_interface_check_timeout(hal_interface_t *hal) {
    if(!hal) return false;

    uint32_t current_time = HAL_GetTick();
    return (current_time - hal->transaction.start_time) > hal->transaction.timeout;
}
*/
