/**
 * @file test_i2c.c
 * @brief Example test that reuses target code for clock, GPIO, I2C, UART init,
 *        and uses the APRS/telemetry approach instead of encodeAPRSFrame().
 */

#include <math.h>  // for fabs()
#include <stdio.h>
#include <stdlib.h>  // for malloc/free
#include <string.h>

#include "gpio.h"  // MX_GPIO_Init
#include "i2c.h"   // MX_I2C1_Init, hi2c1
#include "main.h"  // SystemClock_Config, Error_Handler
#include "stm32u0xx_hal.h"
#include "usart.h"  // MX_USART2_UART_Init, huart2

// APRS / Telemetry stuff
#include "aprs.h"
#include "aprsConfig.h"
#include "ax25.h"  // Contains initFrame(), processFrameVerbose(), etc.

// GNSS driver for MAX-M10S
#include "max_m10s.h"

// External declarations for the global variables in i2c.c, usart.c
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

// Sensor addresses/regs, etc.
#define LTR329_ADDR (0x29 << 1)
#define LTR329_CONTR 0x80
#define LTR329_MEAS_RATE 0x85
#define LTR329_DATA_START 0x88

#define MCP9808_ADDR (0x18 << 1)
#define MCP9808_REG_AMBIENT_TEMP 0x05

#define MAX_M10S_DEFAULT_ADDR (0x42 << 1)

/* Retarget printf to UART2 (optional) */
int _write(int file, char *ptr, int len) {
    (void)file;  // silence unused-parameter warning
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* Helper function to print a string via UART2 */
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
    if (raw & 0x1000) {  // negative bit
        temperature -= 256;
    }
    return temperature;
}

/* --- i2c scan for connected sensors --- */
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

/**
 * @brief Convert a float latitude/longitude into APRS format:
 *        - Lat: "DDMM.mmN" or "DDMM.mmS"
 *        - Lon: "DDDMM.mmE" or "DDDMM.mmW"
 * @param coord  The float coordinate (degrees).
 * @param isLat  true if coord is latitude, false if longitude.
 * @param outStr Buffer for the result, e.g. "4903.50N".
 * @param outSize Size of outStr.
 */
void floatToAprsCoord(float coord, bool isLat, char *outStr, size_t outSize) {
    // Determine hemisphere (N/S for latitude, E/W for longitude)
    char hemi = (coord >= 0)
                    ? (isLat ? 'N' : 'E')
                    : (isLat ? 'S' : 'W');

    float absCoord = fabsf(coord);
    int deg = (int)absCoord;
    float mins = (absCoord - deg) * 60.0f;

    if (isLat) {
        // Format: "DDMM.mmH"
        // e.g. 49 degrees, 3.50 minutes => "4903.50N"
        snprintf(outStr, outSize, "%02d%05.2f%c", deg, mins, hemi);
    } else {
        // Format: "DDDMM.mmH"
        // e.g. 72 degrees, 1.75 minutes => "07201.75W"
        snprintf(outStr, outSize, "%03d%05.2f%c", deg, mins, hemi);
    }
}

/*
 * Example function that uses your APRS library’s telemetry approach.
 * Instead of building a raw text frame, we create a telemetry frame,
 * position frame, etc., then process them via AX.25 or HDLC.
 */
static void buildAndSendAPRSTelemetry(uint16_t lightVal, float tempVal,
                                      float latitude, float longitude) {
    // Convert sensor readings to strings for the APRS telemetry functions
    // Example: "001,"  "002,"  etc.
    // In a real app, you'd format these properly from your sensor data
    char analogValue1[8], analogValue2[8], analogValue3[8], analogValue4[8], analogValue5[8];
    snprintf(analogValue1, sizeof(analogValue1), "%03d,", (int)lightVal);
    snprintf(analogValue2, sizeof(analogValue2), "%03d,", (int)tempVal);
    // For demonstration, fill the rest with dummy values
    strcpy(analogValue3, "003,");
    strcpy(analogValue4, "004,");
    strcpy(analogValue5, "005,");

    // Example digital bits as a string, e.g. "00001100"
    char *digitalValue = "00001100";

    // Example comment
    char *comment = " Hello World!";

    char latStr[16];
    char lonStr[16];
    floatToAprsCoord(latitude, true, latStr, sizeof(latStr));    // For latitude
    floatToAprsCoord(longitude, false, lonStr, sizeof(lonStr));  // For longitude

    printf("Analog1: %s\n", analogValue1);
    printf("Analog2: %s\n", analogValue2);
    printf("Latitude: %s\n", latStr);
    printf("Longitude: %s\n", lonStr);
    // 1) Create a telemetry info frame
    telemetryInfoFrame *tFrame = initTFrame();
    // 2) Fill it with your sensor data
    updateTelemData(
        tFrame,
        (uint8_t *)analogValue1,
        (uint8_t *)analogValue2,
        (uint8_t *)analogValue3,
        (uint8_t *)analogValue4,
        (uint8_t *)analogValue5,
        (uint8_t *)digitalValue,
        comment);
    // 3) Concatenate the data into a final buffer
    concatTelemData(tFrame);

    // 4) Turn the telemetry data into an AX.25 iFrame
    iFrame *frame = initFrame(tFrame->tData, tFrame->tDataSize);
    // 5) Process it into HDLC or NRZI bits
    hdlcFrame infoResult = processFrameVerbose(frame);

    // Print the bitstream or hex if desired
    printf("Telemetry HDLC Bitstream: ");
    for (size_t i = 0; i < infoResult.size; i++) {
        printf("%d", infoResult.nrziBinHdlcFrame[i]);
        printf(", ");
    }
    printf("\n");
    
    // Free the telemetry data
    free(infoResult.nrziBinHdlcFrame);
    free(frame);
    free(tFrame->tData);
    free(tFrame);

    // Optionally, do the same for a Position frame
    positionFrame *pFrame = initPFrame();
    // 1) Convert the float lat/long into APRS strings


    // 2) Pass these strings into updatePositionData
    //    For example, "123456" might be a time or altitude placeholder
    updatePositionData(
        pFrame,
        "123456",  // Could be time or alt, up to you
        latStr,    // e.g. "4903.50N"
        lonStr,    // e.g. "07201.75W"
        "HelloWorld!");
    concatPositionData(pFrame);

    iFrame *posIFrame = initFrame(pFrame->positionFrame, pFrame->positionFrameSize);
    hdlcFrame posResult = processFrameVerbose(posIFrame);

    printf("Position HDLC Bitstream: ");
    for (size_t i = 0; i < posResult.size; i++) {
        printf("%d", posResult.nrziBinHdlcFrame[i]);
        printf(", ");
    }
    printf("\n");

    free(posResult.nrziBinHdlcFrame);
    free(posIFrame);
    free(pFrame->positionFrame);
    free(pFrame);
}

int main(void) {
    // 1) Initialize the HAL and system clock using the target’s code
    HAL_Init();
    SystemClock_Config();   // Provided by main.c in your target code
    MX_GPIO_Init();         // Provided by gpio.c
    MX_I2C1_Init();         // Provided by i2c.c
    MX_USART2_UART_Init();  // Provided by usart.c

    i2c_scan();  // Scan the I2C bus for devices

    // 2) Initialize sensors
    print_uart("Starting I2C sensor test...\r\n");
    LTR329_Init();
    MCP9808_Init();

    // 3) Initialize the GNSS module (MAX-M10S)
    if (max_m10s_init() != UBLOX_OK) {
        print_uart("GPS initialization failed!\r\n");
    } else {
        print_uart("GPS initialized successfully\r\n");
    }

    // 4) Read sensors and build APRS frames in a loop
    while (1) {
        uint16_t lightVal = LTR329_ReadALS();
        float tempVal = MCP9808_ReadTemp();

        // Retrieve GNSS position
        gps_position_t gps_pos;
        float latitude = 0.0f;
        float longitude = 0.0f;

        if ((max_m10s_get_position(&gps_pos) == UBLOX_OK) && gps_pos.valid) {
            // Convert fixed-point format (degrees * 10^7) to float degrees
            latitude = (float)gps_pos.latitude / 10000000.0f;
            longitude = (float)gps_pos.longitude / 10000000.0f;
        } else {
            // If no valid GPS data, use default dummy coordinates
            latitude = 49.05833f;
            longitude = -72.02916f;
        }

        // Instead of encodeAPRSFrame, we now use the telemetry approach:
        buildAndSendAPRSTelemetry(lightVal, tempVal, latitude, longitude);

        // Wait 5 seconds
        HAL_Delay(5000);
    }
}
