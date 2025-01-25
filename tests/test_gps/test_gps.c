/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "u-blox_gnss_MAX-M10S.h"

/* Private variables ---------------------------------------------------------*/
uint8_t rx_char;
ublox_status_e status;
char debug_buff[100];  // Buffer for debug messages

/* Private function prototypes -----------------------------------------------*/
static void print_location_data(void);
static void process_command(uint8_t cmd);

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  #ifdef DEBUG
    debug_print("GPS Test Program\r\n");
    debug_print("Available Commands:\r\n");
    debug_print("'I' - Initialize GPS module\r\n");
    debug_print("'F' - Check GPS Fix status\r\n");
    debug_print("'L' - Get Location data (lat/lon/height)\r\n");
  #endif

  // Wait for initialization command
  while(1) {
    if (HAL_UART_Receive(&huart2, &rx_char, 1, 10) == HAL_OK) {
      if (rx_char == 'I' || rx_char == 'i') {
        #ifdef DEBUG
          debug_print("Initializing GPS module...\r\n");
        #endif
        if (ublox_init() != UBLOX_OK) {
          #ifdef DEBUG
            debug_print("GPS initialization failed!\r\n");
          #endif
          Error_Handler();
        }
        #ifdef DEBUG
          debug_print("GPS initialized successfully\r\n");
          debug_print("Use 'F' to check fix status before requesting location\r\n");
        #endif
        break;
      }
    }
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(1000);
  }

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
  switch(toupper(cmd)) {
    case 'F':  // Check Fix status
      #ifdef DEBUG
        debug_print("Checking GPS fix status...\r\n");
      #endif
      status = ublox_get_nav_status();
      #ifdef DEBUG
        if (status == UBLOX_OK) {
          debug_print("Good GPS fix - Ready for location data\r\n");
        } else {
          debug_print("No GPS fix - Please wait for satellite acquisition\r\n");
        }
      #endif
      break;

    case 'L':  // Get Location data
      #ifdef DEBUG
        debug_print("Requesting location data...\r\n");
      #endif
      
      // First get fresh PVT data
      status = ublox_get_pvt();
      if (status != UBLOX_OK) {
        #ifdef DEBUG
          debug_print("Failed to get location data - Check fix status\r\n");
        #endif
        return;
      }

      // Now get the position data
      print_location_data();
      break;

    default:
      #ifdef DEBUG
        debug_print("Unknown command. Available commands:\r\n");
        debug_print("'F' - Check GPS Fix status\r\n");
        debug_print("'L' - Get Location data\r\n");
      #endif
      break;
  }
}

/**
 * @brief Print location data from GPS
 */
static void print_location_data(void) {
  int32_t lat, lon, height;
  
  status = ublox_get_curr_position(&lat, &lon, &height);
  if (status != UBLOX_OK) {
    #ifdef DEBUG
      debug_print("Error retrieving location data\r\n");
    #endif
    return;
  }

  #ifdef DEBUG
    debug_print("Current Location (raw values):\r\n");
    
    snprintf(debug_buff, sizeof(debug_buff), "  Latitude:  %ld\r\n", lat);
    debug_print(debug_buff);
    
    snprintf(debug_buff, sizeof(debug_buff), "  Longitude: %ld\r\n", lon);
    debug_print(debug_buff);
    
    snprintf(debug_buff, sizeof(debug_buff), "  Altitude:  %ld\r\n", height);
    debug_print(debug_buff);
  #endif
}