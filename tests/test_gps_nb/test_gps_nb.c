/**
 * @file    test_gps_nb.c
 * @brief   Small test for the MAX-M10S GPS driver initialization sequence
 *
 * This test verifies the non-blocking initialization and configuration sequence
 * of the MAX-M10S GPS driver. It focuses on testing:
 *
 * 1. Basic Initialization
 *    - HAL interface setup (I2C and Timer peripherals)
 *    - Device structure initialization
 *    - Protocol selection and validation
 *
 * 2. Configuration Sequence
 *    - State machine transitions during initialization
 *    - UBX protocol enablement over I2C
 *    - NMEA protocol disablement
 *    - Proper handling of acknowledgments from the GPS module
 *
 * 3. Error Handling
 *    - Timeout detection
 *    - Error flag monitoring
 *    - Reset condition detection
 *
 * The test uses a non-blocking approach where the state machine (max_m10s_run)
 * is called repeatedly until either:
 *   a) Configuration completes successfully (config_valid flag set)
 *   b) An error occurs (needs_reset flag set)
 *   c) Timeout occurs (10 seconds elapsed)
 *
 * Test results are printed to the debug console. If tests passes successfully,
 * you will see this output:
 * Final State:
 * Driver State: 1
 * Bus State: 0
 * Flags: 0x0201  // <---config_valid and inited flags set
 *
 * Test complete. All passed!
 */

#include "max_m10s.h"
#include "hal_interface.h"
#include "stm32l4xx_hal.h"
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "logging.h"
#include "gpio.h"

 /* Private types */
typedef struct {
    max_m10s_t gps_dev;
    hal_interface_t hal;
} test_state_t;

static test_state_t test_state = {0};

/* Test logging macros */
#define TEST_PASS(msg, ...) debug_print("✓ " msg "\n", ##__VA_ARGS__)
#define TEST_FAIL(msg, ...) debug_print("✗ " msg "\n", ##__VA_ARGS__)

/**
 * @brief Test the GPS initialization sequence
 */
static void test_gps_init(void) {
    gps_status_e status;
    debug_print("\nTesting GPS Initialization Sequence:\n");
    debug_print("-----------------------------------\n");

    // Step 1: Initialize HAL interface
    test_state.hal.hi2c = &hi2c1;
    test_state.hal.htim = &htim1;

    // Step 2: Initialize GPS device
    status = max_m10s_init(&test_state.gps_dev, &test_state.hal);
    if (status != UBLOX_OK) {
        TEST_FAIL("Basic initialization failed: %d", status);
        return;
    }
    TEST_PASS("Basic initialization");

    // Step 3: Run state machine until configuration complete
    uint32_t timeout = HAL_GetTick() + 10000;  // 10 second timeout
    bool config_success = false;

    while (HAL_GetTick() < timeout) {
        status = max_m10s_run(&test_state.gps_dev);

        if (status != UBLOX_OK) {
            TEST_FAIL("State machine error: %d", status);
            return;
        }

        // Check if configuration completed successfully
        if (test_state.gps_dev.flags.bits.config_valid) {
            config_success = true;
            break;
        }

        // Check for errors
        if (test_state.gps_dev.flags.bits.needs_reset) {
            TEST_FAIL("Configuration failed, reset needed");
            return;
        }

        HAL_Delay(10);  // Small delay between checks
    }

    if (!config_success) {
        TEST_FAIL("Configuration timed out");
        return;
    }

    TEST_PASS("Configuration sequence completed");

    // Print final state
    debug_print("\nFinal State:\n");
    debug_print("  Driver State: %d\n", test_state.gps_dev.state);
    debug_print("  Bus State: %d\n", test_state.gps_dev.bus_state);
    debug_print("  Flags: 0x%04X\n", test_state.gps_dev.flags.all);
}

int main(void) {
    // MCU Configuration
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();

    debug_print("\n=== GPS Init Test ===\n");

    // Run the test
    test_gps_init();

    // Indicate test completion
    debug_print("\nTest complete. ");
    if (test_state.gps_dev.flags.bits.config_valid) {
        debug_print("All passed!\n");
    } else {
        debug_print("Some tests failed.\n");
    }

    while (1) {
        // Blink LED to indicate test completion
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        HAL_Delay(500);
    }
}
