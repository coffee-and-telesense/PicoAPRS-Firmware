/**
 ******************************************************************************
 * @file           : test_integration.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * For a fresh run do cmake --preset Debug && cmake --build build/Debug
 * For a run that already has a debug cmake --build build/Debug --clean-first
 * For flashing/building STM32_Programmer_CLI --connect port=swd --download D:/School/CubeMXtest/u0_integrated_standby/build/Debug/u0_integrated_standby.elf -hardRst -rst --start
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "main.h"
#include "rtc.h"
#include "usart.h"

/* USER CODE BEGIN Includes */
#include <math.h>  //for fabs()
#include <stdio.h>
#include <string.h>

//  // APRSlib stuff
#include "aprs.h"
#include "aprsConfig.h"
#include "ax25.h"

//  // Include GNSS driver for MAX-M10S
#include "max_m10s.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
// Macro defining the prototype for the function that redirects printf output to UART

/* --- Sensor addresses and register definitions --- */
#define LTR329_ADDR (0x29 << 1)  // I2C address of the LTR-329 ambient light sensor (shifted for read/write operations)
#define LTR329_CONTR 0x80        // Control register for configuring the LTR-329 sensor
#define LTR329_MEAS_RATE 0x85    // Register to set the measurement rate and integration time
#define LTR329_DATA_START 0x88   // Register address where ambient light data starts

#define MCP9808_ADDR (0x18 << 1)       // I2C address of the MCP9808 temperature sensor (shifted for read/write operations)
#define MCP9808_REG_AMBIENT_TEMP 0x05  // Register for reading the ambient temperature data

#define MAX_M10S_DEFAULT_ADDR (0x42 << 1)  // Default I2C address of the MAX-M10S GPS module (shifted for read/write)
#define sec 20

/* USER CODE BEGIN PV */
volatile uint8_t buttonPressed = 0;  // Flag to indicate if the button has been pressed (1 = pressed, 0 = not pressed)
volatile uint32_t Threshold = 2854;  // ADC threshold value corresponding to 2.3V (calculated using Vin/Vref * (2^n - 1))
volatile uint32_t value_adc = 0;     // Variable to store the ADC conversion result (raw digital value)
volatile float latitude = 0.0f;
volatile float longitude = 0.0f;
volatile float tempVal = 0.0f;
volatile uint16_t lightVal = 0;
char aprsFrame[100];  // APRS Frame array

/* ================================ */
/*          Helper Functions        */
/* ================================ */

/* --- Retarget printf to UART2 --- */
/* Overrides putchar to send characters through UART2, allowing printf debugging */
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, COM_POLL_TIMEOUT);
    return ch;
}

/* ================================ */
/*       LTR-329 Sensor Functions   */
/* ================================ */
/* --- LTR-329 initialization --- */
static void LTR329_Init(void) {
    uint8_t data[2];
    data[0] = LTR329_CONTR;
    data[1] = 0x01;  // Active mode, gain 1x
    HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

    data[0] = LTR329_MEAS_RATE;
    data[1] = 0x02;  // 2Hz integration, 100ms measurement
    HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

    printf("LTR-329 Initialized!\r\n");
}

/* --- LTR-329 read ambient light --- */
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
/* --- MCP9808 initialization --- */
static void MCP9808_Init(void) {
    printf("MCP9808 Initialized!\r\n");
}

/* --- MCP9808 read temperature in Celsius --- */
static float MCP9808_ReadTemp(void) {
    uint8_t reg = MCP9808_REG_AMBIENT_TEMP;
    uint8_t data[2];
    HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, MCP9808_ADDR, data, 2, HAL_MAX_DELAY);

    uint16_t raw = (data[0] << 8) | data[1];
    float temperature = (raw & 0x0FFF) / 16.0f;
    if (raw & 0x1000) {  // negative bit
        temperature -= 256;
    }
    return temperature;
}
/* ================================ */
/*       LED Blinking Functions     */
/* ================================ */

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
    HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);  // Has to be before clock gets configured

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
    // HAL_GPIO_ALL_LED_OFF();

    printf("\n--- System Booting Up ---\n");
    HAL_Delay(50);
    printf("Initializing Values...\n\n");
    HAL_Delay(50);
}

/* --- i2c scan for connected sensors --- */
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

// External interrupt callback function (called when rising edge detected on button pin)
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == Cap_Intr_Pin) {  // Check if the triggered pin is the push button
        buttonPressed = 1;           // Set flag to indicate button press
    }
}

/* RTC Wakeup Timer Interrupt Handler ----------------------------------------*/
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
    __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);  // Clear RTC wakeup flag
}

/* ================================ */
/*        Miscellaneous Functions   */
/* ================================ */

/* --- Waits for a button press and prints message when pressed --- */
void ButtonTest() {
    while (1) {
        printf("Waiting for Button!\n");
        HAL_Delay(5000);
        if (buttonPressed == 1) {
            printf("Button Pressed!\n");
            buttonPressed = 0;
        }
    }
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

// Reads a single ADC value
// Test function to read and print ADC value to UART
void ADC_READ_TEST() {
    HAL_Delay(1000);  // Initial delay before reading ADC (stabilization time, if needed)

    HAL_ADC_Start(&hadc1);                 // Start ADC conversion on ADC1
    HAL_Delay(10);                         // Delay to allow some time before next reading (optional for testing)
    value_adc = HAL_ADC_GetValue(&hadc1);  // Read the converted value (last result stored in register)
    HAL_ADC_Stop(&hadc1);                  // Stop the ADC after reading (optional in single conversion mode, but good practice)

    // Print the current ADC value and the threshold value to UART (debugging output)
    printf("\nvalue_adc = %lu\nThreshold = %lu\n\n",
           value_adc, Threshold);

    HAL_Delay(2000);  // Delay to allow some time before next reading (optional for testing)
}

/* USER CODE BEGIN 4 */

int main(void) {
    // Initialize all system peripherals and hardware
    INIT();

    HAL_Delay(50);
    HAL_ADCEx_Calibration_Start(&hadc1);  // Calibrate ADC1 (recommended after power-up)

    // Indicate the system has woken up from standby
    printf("System Successfully Booted!\n\n");
    HAL_Delay(500);

    // Read initial ADC value to check battery/capacitor voltage
    printf("Starting ADC Calibration.\n");
    ADC_READ_TEST();

    // Check if the ADC value is above a pre-defined threshold (good power check)
    if (value_adc > Threshold) {
        // If the voltage is good

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

        // Indicate system is entering standby mode
        Enter_Standby_Mode();
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
    while (1) {
        printf("Illegal Area\n");
        HAL_Delay(1000);
    }
    // Normally unreachable, but good practice to return 0 in main
    return 0;
}
