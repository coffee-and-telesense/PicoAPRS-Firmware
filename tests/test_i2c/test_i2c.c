/**
 * @file test_i2c.c
 * @brief Example test that reuses target code for clock, GPIO, I2C, UART init.
 */

#include <stdio.h>
#include <string.h>

#include "gpio.h"  // Has MX_GPIO_Init
#include "i2c.h"   // Has MX_I2C1_Init, and hi2c1 declared
#include "main.h"  // Has declarations of SystemClock_Config, Error_Handler
#include "stm32u0xx_hal.h"
#include "usart.h"  // Has MX_USART2_UART_Init, and huart2 declared

/* --- External declarations for the global variables in i2c.c, usart.c --- */
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

/* --- Sensor addresses/regs, etc. --- */
#define LTR329_ADDR (0x29 << 1)
#define LTR329_CONTR 0x80
#define LTR329_MEAS_RATE 0x85
#define LTR329_DATA_START 0x88

#define MCP9808_ADDR (0x18 << 1)
#define MCP9808_REG_AMBIENT_TEMP 0x05

/* --- Retarget printf to UART2 (optional) --- */
int _write(int file, char *ptr, int len) {
    (void)file;  // to silence unused-parameter warning
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* --- Helper function to print a string via UART2 --- */
static void print_uart(const char *msg) {
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* --- LTR-329 initialization --- */
static void LTR329_Init(void) {
    uint8_t data[2];
    data[0] = LTR329_CONTR;
    data[1] = 0x01;  // Active mode, gain 1x
    HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

    data[0] = LTR329_MEAS_RATE;
    data[1] = 0x02;  // 2Hz integration, 100ms measurement
    HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

    print_uart("LTR-329 Initialized!\r\n");
}

/* --- LTR-329 read ambient light --- */
static uint16_t LTR329_ReadALS(void) {
    uint8_t reg = LTR329_DATA_START;
    uint8_t data[4];
    HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, LTR329_ADDR, data, 4, HAL_MAX_DELAY);
    return (uint16_t)((data[1] << 8) | data[0]);
}

/* --- MCP9808 initialization --- */
static void MCP9808_Init(void) {
    // For basic temperature reads, no special config needed
    print_uart("MCP9808 Initialized!\r\n");
}

/* --- MCP9808 read temperature in Celsius --- */
static float MCP9808_ReadTemp(void) {
    uint8_t reg = MCP9808_REG_AMBIENT_TEMP;
    uint8_t data[2];
    HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c1, MCP9808_ADDR, data, 2, HAL_MAX_DELAY);

    uint16_t raw = (data[0] << 8) | data[1];
    float temperature = (raw & 0x0FFF) / 16.0f;
    if (raw & 0x1000)  // negative bit
    {
        temperature -= 256;
    }
    return temperature;
}

/* --- Simple APRS-like encoding --- */
static void encodeAPRSFrame(uint16_t light_value, float temperature,
                            char *buffer, size_t bufferSize) {
    // Example: "N0CALL>APRS:!4903.50N/07201.75W T=24.50C L=123 lux"
    snprintf(buffer, bufferSize,
             "N0CALL>APRS:!4903.50N/07201.75W T=%.2fC L=%u lux",
             temperature, light_value);
}
// i2c scan for connected sensors
void i2c_scan(void) {
    print_uart("Scanning I2C bus...\r\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 100) == HAL_OK) {
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "Found device at 0x%02X\r\n", addr);
            print_uart(buffer);
        }
    }
    print_uart("Done scanning.\r\n");
}

/* --- Main test function reusing the target init code --- */
int main(void) {
    /* 1) Initialize the HAL and system clock using the target’s code */
    HAL_Init();
    SystemClock_Config();   // Provided by main.c in your target code
    MX_GPIO_Init();         // Provided by gpio.c
    MX_I2C1_Init();         // Provided by i2c.c
    MX_USART2_UART_Init();  // Provided by usart.c

    i2c_scan();  // Call the bus scan function here

    /* 2) Initialize sensors */
    print_uart("Starting I2C sensor test...\r\n");
    LTR329_Init();
    MCP9808_Init();

    /* 3) Read sensors and print APRS-like frame in a loop */
    char aprsFrame[100];
    while (1) {
        uint16_t lightVal = LTR329_ReadALS();
        float tempVal = MCP9808_ReadTemp();

        encodeAPRSFrame(lightVal, tempVal, aprsFrame, sizeof(aprsFrame));

        print_uart(aprsFrame);
        print_uart("\r\n");

        HAL_Delay(5000);  // 5s delay
    }
}
