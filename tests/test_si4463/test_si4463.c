// In main.c
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

#include "si_trx.h"
#include "si_trx_defs.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)


// Function prototypes
void gps_demo_run(void);
// Declare extern private functions from CubeMX
extern void SystemClock_Config(void);

// Global variables
extern SPI_HandleTypeDef hspi1;

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init(); // For printf output

  /* Redirect printf to UART */
  // Add your printf redirection code here if needed

  /* Run the GPS demo application */
  si4463_demo_run();

  /* Infinite loop */
  while (1)
  {
    Error_Handler();
    // This code won't be reached as gps_demo_run has its own infinite loop
  }
}

void si4463_demo_run(void) {
  struct si_frequency_configuration fconfig = {0};

  si_trx_get_frequency_configuration(&fconfig, 144390000);

  printf("Starting CW Si4463 Transmission");

  while (1) {
    si_trx_on(SI_MODEM_MOD_TYPE_CW, &fconfig, 0, 36, SI_FILTER_DEFAULT);
    HAL_Delay(5);
    si_trx_off();
    si_trx_on(SI_MODEM_MOD_TYPE_CW, &fconfig, 0, 0x7f, SI_FILTER_DEFAULT);
    HAL_Delay(5);
    si_trx_off();

    // Toggle LED to show the application is running
    HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
    HAL_Delay(500); // LED blink interval
  }
}

// Redirect printf to UART
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}
