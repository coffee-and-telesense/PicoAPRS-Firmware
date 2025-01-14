#include "main.h"  // Gets us the peripheral handles and pin definitions
#include "u-blox_gnss_MAX-M10S.h"  // Your GPS library

// These are defined in the generated main.c but we need to declare them external 
// so we can call them
extern void SystemClock_Config(void);
extern void MX_GPIO_Init(void);
extern void MX_I2C1_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);

// These handles are defined in main.c and we can use them
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

int main(void)
{
    // Initialize the hardware
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();


    while (1) 
    {
        // Your test code here
        HAL_DELAY(1000);
    }
}