#include "main.h"     // For HAL definitions and Error_Handler
#include "gpio.h"     // For GPIO functions
#include "i2c.h"      // For I2C functions
#include "usart.h"    // For UART functions
#include "test_gps.h" // 
#include <string.h>
#include "u-blox_gnss_MAX-M10S.h"

#ifdef DEBUG 
  #define DEBUG_UART huart2
  #define debug_print(msg) do{ \
    HAL_UART_Transmit(&DEBUG_UART, (uint8_t*)(msg), strlen(msg), 100); \
  } while(0)
#endif

uint8_t rx_char; 
UBLOX_Status_t status;

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //GPS handles init of I2C1
  //MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  while(1) {
    // Wait to receive start char
    if (HAL_UART_Receive(&DEBUG_UART, &rx_char, 1, 10) == HAL_OK){
      if (rx_char == 'S' || rx_char == 's') {
          #ifdef DEBUG
            debug_print("Initing GPS with UBX command...\r\n");
          #endif
          rx_char = 0; 
          break; 
      }
    }
    HAL_Delay(1000);
  }

  if (ublox_init() != UBLOX_OK){
    #ifdef DEBUG
      debug_print("GPS initialization failed!\r\n");
    #endif
    Error_Handler(); 
  } 

  #ifdef DEBUG
    debug_print("System initialized. Send 'T' to trigger NAV status request\r\n");
  #endif
  
  while (1)
  {
    // Test loop
    // Check if we received a character
    if (HAL_UART_Receive(&DEBUG_UART, &rx_char, 1, 10) == HAL_OK) {
      if (rx_char == 'T' || rx_char == 't') {
        #ifdef DEBUG
          debug_print("Requesting NAV status...\r\n");
        #endif
        
        status = ublox_get_nav_status();
        
        #ifdef DEBUG
          if (status == UBLOX_OK) {
            debug_print("NAV status request sent successfully\r\n");
          } else {
            debug_print("NAV status request failed\r\n");
          }
        #endif
      }
    }

    // Blink LED to show the program is running
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(1000);
  }
}