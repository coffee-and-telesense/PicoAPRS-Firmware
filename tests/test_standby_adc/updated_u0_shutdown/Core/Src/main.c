/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * For a fresh run do cmake --preset Debug && cmake --build build/Debug
  * For a run that already has a debug cmake --build build/Debug --clean-first
  * For flashing/building STM32_Programmer_CLI --connect port=swd --download D:/School/CubeMXtest/updated_u0_shutdown/build/Debug/updated_u0_shutdown.elf -hardRst -rst --start
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t buttonPressed = 0;
volatile uint32_t Threshold = 2854; // Vin / Vref * (2^n -1) = 2.3 V /3.3 V * (2^12 -1) = 2854
volatile uint32_t value_adc = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Reads a single ADC value

// Test function to read and print ADC value to UART
void ADC_READ_TEST() {
  HAL_Delay(1000);  // Initial delay before reading ADC (stabilization time, if needed)

  HAL_ADC_Start(&hadc1);  // Start ADC conversion on ADC1
  HAL_Delay(10);  // Delay to allow some time before next reading (optional for testing)
  value_adc = HAL_ADC_GetValue(&hadc1);  // Read the converted value (last result stored in register)
  HAL_ADC_Stop(&hadc1);  // Stop the ADC after reading (optional in single conversion mode, but good practice)

  // Print the current ADC value and the threshold value to UART (debugging output)
  printf("\nvalue_adc = %lu\nThreshold = %lu\n\n", 
         value_adc, Threshold);

  HAL_Delay(2000);  // Delay to allow some time before next reading (optional for testing)
}

// Turns on LED1
void LED1_ON() {
  LED_ON(LED1_GPIO_Port, LED1_Pin);  // Use provided macro to turn on LED1
}

// Turns on LED2
void LED2_ON() {
  LED_ON(LED2_GPIO_Port, LED2_Pin);  // Use provided macro to turn on LED2
}

// Turns on the user LED (probably onboard LED)
void UserLED_ON() {
  LED_ON(UserLED_GPIO_Port, UserLED_Pin);  // Use provided macro to turn on user LED
}

// Turns off LED1
void LED1_OFF() {
  LED_OFF(LED1_GPIO_Port, LED1_Pin);  // Use provided macro to turn off LED1
}

// Turns off LED2
void LED2_OFF() {
  LED_OFF(LED2_GPIO_Port, LED2_Pin);  // Use provided macro to turn off LED2
}

// Turns off the user LED
void UserLED_OFF() {
  LED_OFF(UserLED_GPIO_Port, UserLED_Pin);  // Use provided macro to turn off user LED
}

// Blink the user LED for a given number of seconds
void BLINK_UserLED(int input_sec) {
  UserLED_ON();            // Turn on user LED
  HAL_Delay(input_sec * 1000);   // Wait for the specified time
  UserLED_OFF();           // Turn off user LED
  HAL_Delay(input_sec * 1000);   // Pause before next action
}

// Blink LED1 for a given number of seconds
void BLINK_LED1(int input_sec) {
  LED1_ON();                // Turn on LED1
  HAL_Delay(input_sec * 1000);    // Wait for the specified time
  LED1_OFF();                // Turn off LED1
  HAL_Delay(input_sec * 1000);    // Pause before next action
}

// Blink LED2 for a given number of seconds
void BLINK_LED2(int input_sec) {
  LED2_ON();                // Turn on LED2
  HAL_Delay(input_sec * 1000);    // Wait for the specified time
  LED2_OFF();                // Turn off LED2
  HAL_Delay(input_sec * 1000);    // Pause before next action
}

// Turn off all LEDs (user LED, LED1, LED2)
void HAL_GPIO_ALL_LED_OFF() {
  UserLED_OFF();
  LED2_OFF();
  LED1_OFF();
}

// Turn on all LEDs (user LED, LED1, LED2)
void HAL_GPIO_ALL_LED_ON() {
  UserLED_ON();
  LED2_ON();
  LED1_ON();
}

// Sequentially blink LEDs and then turn all on briefly
void HAL_GPIO_LED_STEP() {
  int input_sec = 1;  // Duration for each blink

  // Sequentially blink each LED
  BLINK_LED2(input_sec);
  BLINK_LED1(input_sec);
  BLINK_UserLED(input_sec);

  // Turn all LEDs on together, hold for a second, then turn all off
  HAL_GPIO_ALL_LED_ON();
  HAL_Delay(input_sec * 1000);
  HAL_GPIO_ALL_LED_OFF();
  HAL_Delay(input_sec * 1000);
}

// Initializes system hardware and peripherals
void INIT() {
  HAL_Init();                  // Initialize HAL library
  SystemClock_Config();        // Configure system clock
  MX_GPIO_Init();               // Initialize GPIOs
  // HAL_MspInit();                // Initialize low-level hardware (clocks, etc.)
  SystemInit();                 // System-level initialization
  // MX_RTC_Init();                // Initialize RTC peripheral
  MX_ADC1_Init();               // Initialize ADC1 peripheral
  MX_USART2_UART_Init();        // Initialize USART2 for debugging/printf

  HAL_Delay(1000);                      // Delay to allow system to stabilize
  HAL_GPIO_ALL_LED_OFF();               // Ensure all LEDs are off after init
}

// External interrupt callback function (called when rising edge detected on button pin)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == PushButton_Pin) {  // Check if the triggered pin is the push button
    buttonPressed = 1;               // Set flag to indicate button press
  }
}

// Redirects printf() to use USART2 for debugging
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, COM_POLL_TIMEOUT);  // Send single character to USART2
  return ch;  // Return character to satisfy function signature
}

/* RTC Wakeup Timer Interrupt Handler ----------------------------------------*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);  // Clear RTC wakeup flag
}

void Enter_Standby_Mode(void){
  // Wakeup Time = WKUP_Counter * (RTCCLK_Div / 32kHz)
  
  //HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x500B, RTC_WAKEUPCLOCK_RTCCLK_DIV16, 0); // 10 sec 

  printf("Entering Standby Mode now...\n");
  HAL_PWR_EnterSTANDBYMode();

  // The MCU does not return from this function unless reset
  printf("This should never be printed unless standby mode fails.\n");
}

void ButtonTest() {
  while(1) {
    printf("Waiting for Button!\n");
    HAL_Delay(5000);
    if (buttonPressed == 1) {
      printf("Button Pressed!\n");
      buttonPressed = 0;
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int main(void) {
  // Initialize all system peripherals and hardware
  INIT();
  printf("\n--- System Booting Up ---\n");
  HAL_Delay(50);
  // Initialize variables or system state (optional message for debugging)
  printf("Initializing Values...\n\n");
  HAL_Delay(50);
  printf("Starting ADC Calibration.\n");
  HAL_Delay(50);
  HAL_ADCEx_Calibration_Start(&hadc1);  // Calibrate ADC1 (recommended after power-up)
  // Ensure all LEDs are off at the start
  HAL_GPIO_ALL_LED_OFF();
  // while(1) {
  //   ADC_READ_TEST();
  // }
  // Indicate the system has woken up from standby
  printf("Woken up from standby!\n\n");
  HAL_Delay(500);
  
  // Run LED test sequence to verify all LEDs are functional
  printf("- - - - - - - - - - - - - -\n");
  printf("Making Sure All LEDs work.\n");
  printf("- - - - - - - - - - - - - -\n");
  HAL_GPIO_LED_STEP();
  HAL_Delay(1000);

  // Read initial ADC value to check battery/capacitor voltage
  ADC_READ_TEST();

  // Check if the ADC value is above a pre-defined threshold (good power check)
  if (value_adc > Threshold) {
    // If the voltage is good, turn on both LEDs
    LED1_ON();
    HAL_Delay(1000);
    LED2_ON();
    HAL_Delay(5000);  // Hold LEDs on for 5 seconds

    // Loop waiting for user interaction (button press) to confirm system is ready
    while (1) {
      printf("Waiting for good ADC value (aka button) \n\n");
      HAL_Delay(2000);

      // Check if the button was pressed (set by external interrupt callback)
      if (buttonPressed == 1) {
        // Confirm good capacitor values via button press
        printf("Capacitor values are good! (Button Pressed!)\n");
        HAL_Delay(1000);

        // Clear the button press flag
        buttonPressed = 0;
        break;  // Exit the while loop
      }
    }

    // After button confirmation, turn off all LEDs
    HAL_GPIO_ALL_LED_OFF();

    // Indicate system is entering standby mode
    printf("Going into standby mode!\n\nZZZZZ\n\n");
    HAL_Delay(5000);

    /* Configure RTC Wakeup */
    printf("Initializing RTC...\n");
    MX_RTC_Init();
    printf("RTC Initialized.\n");
    printf("Configuring Timer to %ds...\n",sec);

    /* Enter Standby Mode */
    printf("Entering Standby Mode...\n");
    Enter_Standby_Mode();
    // System will restart at main() when woken up
  }

  // If ADC voltage is too low (bad capacitor voltage), turn on user LED
  // Run LED step sequence to provide visual feedback
  HAL_Delay(500);
  UserLED_ON();
  printf("Bad voltage!\n\n");
  
  // Hold warning state for 5 seconds
  HAL_Delay(2000);
  
  // Turn off all LEDs before entering standby mode
  HAL_GPIO_ALL_LED_OFF();
  
  // Indicate system is entering standby due to low voltage
  printf("Going into standby mode!\n\nZZZZZ\n");
  HAL_Delay(5000);

  /* Configure RTC Wakeup */
  printf("Initializing RTC...\n");
  MX_RTC_Init();
  printf("RTC Initialized.\n");
  printf("Configuring Timer to %ds...\n",sec);

  /* Enter Standby Mode */
  printf("Entering Standby Mode...\n");
  Enter_Standby_Mode();
  // System will restart at main() when woken up
  while(1){
    printf("Illegal Area\n");
    HAL_Delay(1000);
  }
  // Normally unreachable, but good practice to return 0 in main
  return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     (void)file;
     (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
