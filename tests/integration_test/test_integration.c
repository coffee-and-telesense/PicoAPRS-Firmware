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
  * For flashing/building STM32_Programmer_CLI --connect port=swd --download D:/School/CubeMXtest/u0_integrated_standby/build/Debug/u0_integrated_standby.elf -hardRst -rst --start
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>  //for fabs()
#include <string.h>

//  // APRSlib stuff
 #include "aprs.h"
 #include "aprsConfig.h"
 #include "ax25.h"
 
//  // Include GNSS driver for MAX-M10S
 #include "max_m10s.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)  
// Macro defining the prototype for the function that redirects printf output to UART  

/* --- Sensor addresses and register definitions --- */
#define LTR329_ADDR (0x29 << 1)  // I2C address of the LTR-329 ambient light sensor (shifted for read/write operations)
#define LTR329_CONTR 0x80        // Control register for configuring the LTR-329 sensor
#define LTR329_MEAS_RATE 0x85    // Register to set the measurement rate and integration time
#define LTR329_DATA_START 0x88   // Register address where ambient light data starts

#define MCP9808_ADDR (0x18 << 1)  // I2C address of the MCP9808 temperature sensor (shifted for read/write operations)
#define MCP9808_REG_AMBIENT_TEMP 0x05  // Register for reading the ambient temperature data

#define MAX_M10S_DEFAULT_ADDR (0x42 << 1)  // Default I2C address of the MAX-M10S GPS module (shifted for read/write)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t buttonPressed = 0;  // Flag to indicate if the button has been pressed (1 = pressed, 0 = not pressed)
volatile uint32_t Threshold = 2854;  // ADC threshold value corresponding to 2.3V (calculated using Vin/Vref * (2^n - 1))
volatile uint32_t value_adc = 0;     // Variable to store the ADC conversion result (raw digital value)
volatile float latitude = 0.0f;
volatile float longitude = 0.0f;
volatile float tempVal = 0.0f;
volatile uint16_t lightVal = 0;
char aprsFrame[100];                 // APRS Frame array
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// void BLINK_LED1(int input_sec);
// void BLINK_LED2(int input_sec);
// void BLINK_UserLED(int input_sec);
void i2c_scan(void);
int max_m10s_init(void);
// void LED_OFF(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
// void LED_ON(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ================================ */
/*          Helper Functions        */
/* ================================ */

/* --- Retarget printf to UART2 --- */
/* Overrides putchar to send characters through UART2, allowing printf debugging */
PUTCHAR_PROTOTYPE {
  HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, COM_POLL_TIMEOUT);
  return ch;
}

// void LED_OFF(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
//   HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
// }

// void LED_ON(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
//   HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
// }

/* ================================ */
/*       LTR-329 Sensor Functions   */
/* ================================ */

/* --- Initializes the LTR-329 ambient light sensor --- */
/* Configures the sensor to active mode with a 1x gain and a 2Hz measurement rate */
static void LTR329_Init(void) {
  uint8_t data[2];
  data[0] = LTR329_CONTR;
  data[1] = 0x01;  // Set to active mode with gain 1x
  HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

  data[0] = LTR329_MEAS_RATE;
  data[1] = 0x02;  // Set measurement rate to 2Hz, integration time 100ms
  HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

  printf("LTR-329 Initialized!\r\n");
}

/* --- Reads ambient light data from the LTR-329 sensor --- */
/* Retrieves 4 bytes of ALS data from the sensor and processes it into a readable format */
static uint16_t LTR329_ReadALS(void) {
  uint8_t reg = LTR329_DATA_START;
  uint8_t data[4];
  HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, LTR329_ADDR, data, 4, HAL_MAX_DELAY);
  return (uint16_t)((data[1] << 8) | data[0]);
}

/* ================================ */
/*       MCP9808 Sensor Functions   */
/* ================================ */

/* --- Initializes the MCP9808 temperature sensor --- */
/* Placeholder function for initialization sequence */
static void MCP9808_Init(void) {
  printf("MCP9808 Initialized!\r\n");
}

/* --- Reads temperature in Celsius from the MCP9808 sensor --- */
/* Retrieves raw temperature data, processes it, and returns the final value */
static float MCP9808_ReadTemp(void) {
  uint8_t reg = MCP9808_REG_AMBIENT_TEMP;
  uint8_t data[2];
  HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, MCP9808_ADDR, data, 2, HAL_MAX_DELAY);

  uint16_t raw = (data[0] << 8) | data[1];
  float temperature = (raw & 0x0FFF) / 16.0f;
  if (raw & 0x1000)  // Check if temperature is negative
      temperature -= 256;

  return temperature;
}

/* ================================ */
/*       LED Control Functions      */
/* ================================ */

// /* --- Functions to turn individual LEDs ON --- */
// void LED1_ON() { LED_ON(LED1_GPIO_Port, LED1_Pin); }
// void LED2_ON() { LED_ON(LED2_GPIO_Port, LED2_Pin); }
// void UserLED_ON() { LED_ON(UserLED_GPIO_Port, UserLED_Pin); }

// /* --- Functions to turn individual LEDs OFF --- */
// void LED1_OFF() { LED_OFF(LED1_GPIO_Port, LED1_Pin); }
// void LED2_OFF() { LED_OFF(LED2_GPIO_Port, LED2_Pin); }
// void UserLED_OFF() { LED_OFF(UserLED_GPIO_Port, UserLED_Pin); }

// /* --- Turn off all LEDs --- */
// void HAL_GPIO_ALL_LED_OFF() {
//   UserLED_OFF();
//   LED2_OFF();
//   LED1_OFF();
// }

// /* --- Turn on all LEDs --- */
// void HAL_GPIO_ALL_LED_ON() {
//   UserLED_ON();
//   LED2_ON();
//   LED1_ON();
// }

// /* --- Sequentially blink LEDs and then turn all on briefly --- */ 
// void HAL_GPIO_LED_STEP() {
//   int input_sec = 1;  // Duration for each blink

//   // Sequentially blink each LED
//   BLINK_LED2(input_sec);
//   BLINK_LED1(input_sec);
//   BLINK_UserLED(input_sec);

//   // Turn all LEDs on together, hold for a second, then turn all off
//   HAL_GPIO_ALL_LED_ON();
//   HAL_Delay(input_sec * 1000);
//   HAL_GPIO_ALL_LED_OFF();
//   HAL_Delay(input_sec * 1000);
// }

// /* --- Tests LEDs to make sure they all work --- */
// void LED_TEST() {
//   printf("- - - - - - - - - - - - - -\n");
//   printf("Making Sure All LEDs work.\n");
//   printf("- - - - - - - - - - - - - -\n");
//   HAL_GPIO_LED_STEP();
//   HAL_Delay(1000);
// }



/* ================================ */
/*       LED Blinking Functions     */
/* ================================ */

// /* --- Blink the User LED for a given number of seconds --- */
// void BLINK_UserLED(int input_sec) {
//   UserLED_ON();
//   HAL_Delay(input_sec * 1000);
//   UserLED_OFF();
//   HAL_Delay(input_sec * 1000);
// }

// /* --- Blink LED1 for a given number of seconds --- */
// void BLINK_LED1(int input_sec) {
//   LED1_ON();
//   HAL_Delay(input_sec * 1000);
//   LED1_OFF();
//   HAL_Delay(input_sec * 1000);
// }

// /* --- Blink LED2 for a given number of seconds --- */
// void BLINK_LED2(int input_sec) {
//   LED2_ON();
//   HAL_Delay(input_sec * 1000);
//   LED2_OFF();
//   HAL_Delay(input_sec * 1000);
// }

/* ================================ */
/*       System Initialization      */
/* ================================ */

/* --- Initialize system and peripherals --- */
void INIT() {
  printf("Initializing HAL...\n");
  HAL_Init();
  printf("Initializing RTC...\n");
  MX_RTC_Init();
  /* Disable Wakeup Timer before setting */
  printf("Disabling RTC Timer.\n\n");
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc); // Has to be before clock gets configured

  SystemClock_Config();
  printf("Initializing GPIO Ports...\n");
  MX_GPIO_Init();
  printf("Initializing ADC...\n");
  MX_ADC1_Init();
  printf("Initializing USART...\n");
  MX_USART2_UART_Init();
  printf("Initializing I2C...\n");
  MX_I2C1_Init();        
  HAL_Delay(50);
  HAL_GPIO_ALL_LED_OFF();
  
  printf("\n--- System Booting Up ---\n");
  HAL_Delay(50);
  printf("Initializing Values...\n\n");
  HAL_Delay(50);
}

void SensorINIT() {
  i2c_scan();  // Scan the I2C bus for devices
  /* Initialize sensors */
  printf("Starting I2C sensor test...\r\n");
  LTR329_Init();
  MCP9808_Init();

  /* Initialize the GNSS module (MAX-M10S)
    * Note: The default I2C address for the MAX-M10S is 0x42.
    * Ensure that your max_m10s driver is configured accordingly.
    */
  if (max_m10s_init() != UBLOX_OK) {
    printf("GPS initialization failed!\r\n");
  } 
  else {
    printf("GPS initialized successfully\r\n");
  }

}

// External interrupt callback function (called when rising edge detected on button pin)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == Cap_Intr_Pin) {  // Check if the triggered pin is the push button
    buttonPressed = 1;               // Set flag to indicate button press
  }
}

/* RTC Wakeup Timer Interrupt Handler ----------------------------------------*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);  // Clear RTC wakeup flag
}

/* --- APRS-like frame encoding using real GNSS data --- */
static void encodeAPRSFrame(uint16_t light_value, float temperature,
  float latitude, float longitude,
  char *buffer, size_t bufferSize) {
  // Convert latitude: degrees & minutes with N/S indicator
  int lat_deg = (int)fabs(latitude);
  float lat_min = (fabs(latitude) - lat_deg) * 60.0f;
  char ns = (latitude >= 0) ? 'N' : 'S';

  // Convert longitude: degrees & minutes with E/W indicator
  int lon_deg = (int)fabs(longitude);
  float lon_min = (fabs(longitude) - lon_deg) * 60.0f;
  char ew = (longitude >= 0) ? 'E' : 'W';

  // Format the APRS string (example):
  // "N0CALL>APRS:!4903.50N/07201.75W T=24.50C L=123 lux"
  snprintf(buffer, bufferSize,
  "N0CALL>APRS:!%02d%05.2f%c/%03d%05.2f%c T=%.2fC L=%u lux",
  lat_deg, lat_min, ns, lon_deg, lon_min, ew,
  temperature, light_value);
}
/* ================================ */
/*        Miscellaneous Functions   */
/* ================================ */

/* --- Scan I2C bus for connected devices --- */
void i2c_scan(void) {
  printf("Scanning I2C bus...\r\n");
  for (uint8_t addr = 1; addr < 127; addr++) {
      if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 100) == HAL_OK) {
          char buffer[64];
          snprintf(buffer, sizeof(buffer), "Found device at 0x%02X\r\n", addr);
          printf(buffer);
      }
  }
  printf("Done scanning.\r\n");
}

/* --- Waits for a button press and prints message when pressed --- */
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

void RTC_TEST() {
  /* Disable Wakeup Timer before setting */
  // printf("Disabling RTC Timer.\n\n");
  // HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
   // Indicate system is entering standby mode
   Enter_Standby_Mode();
}

/* --- Enter standby mode with RTC wake-up --- */
void Enter_Standby_Mode() {
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 20, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0);
  printf("Wakeup Timer Set.\n");
  printf("Configuring Timer to %ds...\n", sec);
  printf("Entering Standby Mode...\n");
  HAL_PWR_EnterSTANDBYMode();
  printf("This should never be printed unless standby mode fails.\n");
}

/* --- Displays LEDs for when there's good ADC Value */
// void Display_Good_ADC() {
//   LED1_ON();
//   HAL_Delay(1000);
//   LED2_ON();
//   HAL_Delay(5000);  // Hold LEDs on for 5 seconds
// }

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

void ReadSensor() {
  lightVal = LTR329_ReadALS();
  float tempVal = MCP9808_ReadTemp();

  // Commented out until GPS module is created
  // Retrieve GNSS position
  gps_position_t gps_pos;
  
  if ((max_m10s_get_position(&gps_pos) == UBLOX_OK) && gps_pos.valid) {
      // Convert fixed-point format (degrees * 10^7) to float degrees
      latitude  = (float)gps_pos.latitude  / 10000000.0f;
      longitude = (float)gps_pos.longitude / 10000000.0f;
  } else {
      // If no valid GPS data, use default dummy coordinates (or handle as needed)
      latitude  = 49.05833f;   // Example dummy latitude
      longitude = -72.02916f;  // Example dummy longitude
  }
}

void Read_APRS() {
  printf(aprsFrame);
  printf("\r\n");
  HAL_Delay(5000);  // 5-second delay
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  
  HAL_Delay(50);
  HAL_ADCEx_Calibration_Start(&hadc1);  // Calibrate ADC1 (recommended after power-up)
  // Ensure all LEDs are off at the start
  //HAL_GPIO_ALL_LED_OFF();
  
  // Indicate the system has woken up from standby
  printf("System Successfully Booted!\n\n");
  HAL_Delay(500);

  // Run LED test sequence to verify all LEDs are functional
  // LED_TEST();

  // Read initial ADC value to check battery/capacitor voltage
  printf("Starting ADC Calibration.\n");
  ADC_READ_TEST();

  // Check if the ADC value is above a pre-defined threshold (good power check)
  if (value_adc > Threshold) {
    // If the voltage is good, turn on both LEDs
    Display_Good_ADC();

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

    // Initializes and reads the sensors
    SensorINIT();
    ReadSensor(); 

    // Build the APRS frame with sensor and GNSS data
    encodeAPRSFrame(lightVal, tempVal, latitude, longitude, aprsFrame, sizeof(aprsFrame));
    
    // Reads the APRS Packet
    Read_APRS();
    
    // Indicate system is entering standby mode
    Enter_Standby_Mode();
    // System will restart at main() when woken up
  }

  // If ADC voltage is too low (bad capacitor voltage), turn on user LED
  // Run LED step sequence to provide visual feedback
  HAL_Delay(500);
  printf("Bad voltage!\n\n");
  
  // Hold warning state for 5 seconds
  HAL_Delay(2000);
  
  // Indicate system is entering standby due to low voltage
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
