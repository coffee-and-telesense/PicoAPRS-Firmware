#include "main.h"     // For HAL definitions and Error_Handler
#include "gpio.h"     // For GPIO functions
#include "i2c.h"      // For I2C functions
#include "usart.h"    // For UART functions
#include "test_gps.h" // 

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  /* Your test code here */
  while (1)
  {
    // Test loop
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    HAL_Delay(1000);
  }
}