// In main.c
#include <stdio.h>

#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "usart.h"

// Include GPS driver headers
#include "driver/max_m10s.h"
#include "gps_types.h"
#include "protocols/ubx/ubx.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

static void print_status(const char* message, gps_status_e status);

// Function prototypes
void gps_demo_run(void);
// Declare extern private functions from CubeMX
extern void SystemClock_Config(void);

// Global variables
extern I2C_HandleTypeDef hi2c1;
static max_m10s_dev_s gps_dev;
static max_m10s_init_s gps_init;

int main(void) {
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();  // For printf output

    /* Redirect printf to UART */
    // Add your printf redirection code here if needed

    /* Run the GPS demo application */
    gps_demo_run();

    /* Infinite loop */
    while (1) {
        Error_Handler();
        // This code won't be reached as gps_demo_run has its own infinite loop
    }
}

/**
 * @brief Main application entry point
 */
void gps_demo_run(void) {
    gps_status_e status;
    printf("GPS Demo Application Starting...\r\n");

    // Initialize GPS configuration
    gps_init.hi2c = &hi2c1;
    gps_init.device_address = 0x42;
    gps_init.timeout_ms = 1000;
#ifdef NON_BLOCKING
    gps_init.transmit = HAL_I2C_Master_Transmit_IT;
    gps_init.receive = HAL_I2C_Master_Receive_IT;
    printf("Running in NON_BLOCKING mode\r\n");
#else
    gps_init.transmit = HAL_I2C_Master_Transmit;
    gps_init.receive = HAL_I2C_Master_Receive;
    printf("Running in BLOCKING mode\r\n");
#endif
    gps_init.delay_blocking = HAL_Delay;

    printf("Initializing GPS device...\r\n");

    // Initialize the GPS device
    status = max_m10s_init(&gps_dev, &gps_init);
    print_status("GPS initialization", status);

    if (status != UBLOX_OK) {
        printf("GPS initialization failed with status: 0x%02X\r\n", status);
        return;
    }
    printf("GPS device initialized successfully\r\n");

    printf("Configuring measurement rate...\r\n");
    // Configure measurement rate to 10 Hz (100ms)
    status = max_m10s_config_meas_rate(&gps_dev, 100);
    print_status("Configuring measurement rate", status);
    if (status != UBLOX_OK) {
        printf("Failed to configure measurement rate with status: 0x%02X\r\n", status);
        return;
    }
    printf("Measurement rate configured successfully\r\n");

    // Main application loop
    printf("Entering main loop...\r\n");
    uint32_t last_pvt_request_time = 0;
    uint32_t pvt_interval = 5000;  // Request PVT data every 5 seconds

    while (1) {
        uint32_t current_time = HAL_GetTick();

        // Check if it's time to request PVT data
        if (current_time - last_pvt_request_time >= pvt_interval) {
            last_pvt_request_time = current_time;

            printf("\r\n--- Requesting PVT data ---\r\n");

            // Step 1: Send PVT command
            status = max_m10s_command(&gps_dev, GPS_CMD_PVT);
            print_status("PVT command", status);

            if (status != UBLOX_OK) {
                printf("Failed to send PVT command\r\n");
                continue;
            }

#ifdef NON_BLOCKING
            // In non-blocking mode, we need to wait for the I2C transaction to complete
            printf("Waiting for I2C command transaction to complete...\r\n");
            gps_status_e wait_status = i2c_wait_for_complete(&gps_dev);
            print_status("I2C wait", wait_status);
#endif

            // Add a small delay between command and read
            HAL_Delay(100);

            // Step 2: Read PVT data
            printf("Reading PVT data...\r\n");
            status = max_m10s_read(&gps_dev);
            print_status("PVT read", status);

#ifdef NON_BLOCKING
            // In non-blocking mode, we need to wait for the I2C transaction to complete
            printf("Waiting for I2C read transaction to complete...\r\n");
            wait_status = i2c_wait_for_complete(&gps_dev);
            print_status("I2C wait", wait_status);
#endif

            // Validate the received data
            status = max_m10s_validate_response(&gps_dev, GPS_CMD_PVT);

            if (status == UBLOX_OK) {
                printf("PVT data received. First few bytes of response: ");
                // Print the first 8 bytes of the received data for verification
                for (int i = 0; i < 8 && i < gps_dev.rx_size; i++) {
                    printf("0x%02X ", gps_dev.rx_buffer[i]);
                }
                printf("\r\n");
            } else {
                printf("Failed to read PVT data\r\n");
            }
        }

        // Toggle LED to show the application is running
        HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
        HAL_Delay(500);  // LED blink interval
    }
}

static void print_status(const char* message, gps_status_e status) {
    const char* status_str;

    switch (status) {
        case UBLOX_OK:
            status_str = "OK";
            break;
        case UBLOX_ERROR:
            status_str = "ERROR";
            break;
        case UBLOX_TIMEOUT:
            status_str = "TIMEOUT";
            break;
        case UBLOX_INVALID_PARAM:
            status_str = "INVALID PARAM";
            break;
        case UBLOX_CHECKSUM_ERR:
            status_str = "CHECKSUM ERROR";
            break;
        case UBLOX_I2C_ERROR:
            status_str = "I2C ERROR";
            break;
        default:
            status_str = "UNKNOWN";
            break;
    }

    printf("%s: %s (0x%02X)\r\n", message, status_str, status);
}

// Redirect printf to UART
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
    return ch;
}

#ifdef NON_BLOCKING
// Callback functions for non-blocking I2C
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef* hi2c) {
    (void)hi2c;  // Unused parameter
    printf("HAL_I2C_MasterTxCpltCallback called\r\n");
    // This function is called when the Master Transmit completes
    // You can set a flag here to indicate completion but no flags are needed for this example
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef* hi2c) {
    (void)hi2c;  // Unused parameter
    printf("HAL_I2C_MasterRxCpltCallback called\r\n");
    // This function is called when the Master Receive completes
    // You can set a flag here to indicate completion
    // You can set a flag here to indicate completion but no flags are needed for this example
}
#endif
