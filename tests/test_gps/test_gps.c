/*******************************************************************************
* @file    test_gps.c
* @brief   Test application for the MAX-M10S GPS blocking driver
*
* @details The application uses a serial of commands to interact with the MAX-M10S; see
*          print_help() for a list of available commands. The application demonstrates
*          how to:
*          - Initialize the GPS module
*          - Check GPS fix status
*          - Retrieve and display position data (lat/lon/altitude)
*          - Retrieve and display UTC time
*          - Reset the module
*
*          The application shows proper handling of:
*          - Driver initialization sequence
*          - Frame lifecycle management
*          - Fixed-point coordinate conversion
*          - Error handling
*          - Data validation
*
* @note    Designed for STM32L432KC Nucleo board with:
*          - MAX-M10S connected via I2C1
*          - Debug output via UART2
*          - LED on PA5 for heartbeat
*          - Coordinates are in fixed-point format (degrees * 10^7)
*
* @author  Reece Wayt
* @date    February 20, 2025
* @version 1.1
*
* @dependencies
*   - STM32 HAL
*   - MAX-M10S GPS Blocking Driver
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "logging.h"
#include <ctype.h>
#include "max_m10s.h"
#include <stdlib.h>

/* Private types -------------------------------------------------------------*/
typedef struct {
    bool gps_initialized;
} app_state_t;

/* Private variables ---------------------------------------------------------*/
static app_state_t app_state = {0};
static uint8_t rx_char;

/* Private function prototypes -----------------------------------------------*/
static void print_position_data(const gps_position_t* position);
static void print_time_data(const gps_time_t* time);
static void process_command(uint8_t cmd);
static void print_help(void);

/**
 * @brief Main application entry point
 */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    //MX_USART1_UART_Init(); // Uncomment if using UART1
    MX_USART2_UART_Init();
    MX_I2C1_Init();  // Make sure I2C is initialized before GPS

    #ifdef DEBUG
        debug_print("\r\n=== GPS Test Program v%d.%d ===\r\n", 1, 1);
        print_help();
    #endif

    /* Main loop --------------------------------------------------------------*/
    while (1) {
        // Check for commands
        if (HAL_UART_Receive(&huart2, &rx_char, 1, 10) == HAL_OK) {
            process_command(rx_char);
        }

        // Heartbeat LED
        HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
        HAL_Delay(500);
    }
}

/**
 * @brief Process received commands
 * @param cmd Command character received
 */
static void process_command(uint8_t cmd) {
    gps_status_e status;
    gps_data_t gps_data;

    switch(toupper(cmd)) {
        case 'I':  // Initialize GPS
            #ifdef DEBUG
                debug_print("Initializing GPS module...\r\n");
            #endif

            status = max_m10s_init();
            if (status != UBLOX_OK) {
                #ifdef DEBUG
                    debug_print("GPS initialization failed with status: %d\r\n", status);
                #endif
                return;
            }

            app_state.gps_initialized = true;
            #ifdef DEBUG
                debug_print("GPS initialized successfully\r\n");
                debug_print("Use 'F' to check fix status before requesting location\r\n");
            #endif
            break;

        case 'F':  // Check Fix status
            if (!app_state.gps_initialized) {
                debug_print("GPS not initialized. Use 'I' first.\r\n");
                return;
            }

            #ifdef DEBUG
                debug_print("Checking GPS fix status...\r\n");
            #endif

            status = max_m10s_get_nav_status();
            switch(status) {
                case UBLOX_OK:
                    debug_print("Good GPS fix - Ready for location data\r\n");
                    break;
                case UBLOX_NO_FIX:
                    debug_print("No GPS fix - Please wait for satellite acquisition\r\n");
                    break;
                case UBLOX_INVALID_TIME:
                    debug_print("GPS time not valid yet\r\n");
                    break;
                default:
                    debug_print("Error checking fix status: %d\r\n", status);
                    break;
            }
            break;

        case 'L':  // Get Location data
            if (!app_state.gps_initialized) {
                debug_print("GPS not initialized. Use 'I' first.\r\n");
                return;
            }

            #ifdef DEBUG
                debug_print("Requesting location data...\r\n");
            #endif

            status = max_m10s_get_position(&gps_data.position);
            if (status != UBLOX_OK) {
                #ifdef DEBUG
                    debug_print("Failed to get position data: %d\r\n", status);
                #endif
                return;
            }

            print_position_data(&gps_data.position);
            break;

        case 'T':  // Get Time data
            if (!app_state.gps_initialized) {
                debug_print("GPS not initialized. Use 'I' first.\r\n");
                return;
            }

            #ifdef DEBUG
                debug_print("Requesting time data...\r\n");
            #endif

            status = max_m10s_get_time(&gps_data.time);
            if (status != UBLOX_OK) {
                #ifdef DEBUG
                    debug_print("Failed to get time data: %d\r\n", status);
                #endif
                return;
            }

            print_time_data(&gps_data.time);
            #ifdef DEBUG
                debug_print("Time data received successfully, freeing frame\r\n");
            #endif
            max_m10s_free_frame();
            break;

        case 'R':  // Reset GPS
            if (!app_state.gps_initialized) {
                debug_print("GPS not initialized. Use 'I' first.\r\n");
                return;
            }

            #ifdef DEBUG
                debug_print("Resetting GPS module...\r\n");
            #endif

            status = max_m10s_reset();
            if (status != UBLOX_OK) {
                debug_print("Reset failed with status: %d\r\n", status);
                return;
            }

            app_state.gps_initialized = false;
            debug_print("GPS reset successful. Please re-initialize.\r\n");
            break;

        case 'H':  // Help
            print_help();
            break;

        default:
            #ifdef DEBUG
                debug_print("Unknown command '%c'. Press 'H' for help.\r\n", cmd);
            #endif
            break;
    }
}

/**
 * @brief Print position data from GPS
 */
static void print_position_data(const gps_position_t* position) {
    if (!position->valid) {
        debug_print("Position data not valid!\r\n");
        return;
    }

    // We are using fixed-point format for latitude, longitude, and altitude
    #ifdef DEBUG
        // Print latitude (degrees * 10^7)
        int32_t lat_deg = position->latitude / 10000000;
        uint32_t lat_dec = labs(position->latitude % 10000000);

        // Print longitude (degrees * 10^7)
        int32_t lon_deg = position->longitude / 10000000;
        uint32_t lon_dec = labs(position->longitude % 10000000);

        // Print altitude (mm to m)
        int32_t alt_m = position->altitude / 1000;
        uint32_t alt_mm = labs(position->altitude % 1000);

        debug_print("\r\nCurrent Position:\r\n");
        debug_print("  Latitude:   %ld.%07lu°\r\n", lat_deg, lat_dec);
        debug_print("  Longitude:  %ld.%07lu°\r\n", lon_deg, lon_dec);
        debug_print("  Altitude:   %ld.%03lu m\r\n", alt_m, alt_mm);
        debug_print("  Satellites: %d\r\n", position->satellites);
        debug_print("  Fix Type:   %d\r\n", position->fix_type);
    #endif
}

/**
 * @brief Print time data from GPS
 */
static void print_time_data(const gps_time_t* time) {
    if (!time->valid) {
        debug_print("Time data not valid!\r\n");
        return;
    }

    #ifdef DEBUG
        debug_print("\r\nCurrent UTC Time:\r\n");
        debug_print("  Date: %04d-%02d-%02d\r\n",
                   time->year, time->month, time->day);
        debug_print("  Time: %02d:%02d:%02d\r\n",
                   time->hour, time->minute, time->second);
    #endif
}

/**
 * @brief Print help menu
 */
static void print_help(void) {
    debug_print("\r\nAvailable Commands:\r\n");
    debug_print("  'I' - Initialize GPS module\r\n");
    debug_print("  'F' - Check GPS Fix status\r\n");
    debug_print("  'L' - Get Location data\r\n");
    debug_print("  'T' - Get Time data\r\n");
    debug_print("  'R' - Reset GPS module\r\n");
    debug_print("  'H' - Show this help menu\r\n\r\n");
}
