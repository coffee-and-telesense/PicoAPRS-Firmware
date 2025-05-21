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
#include "logging.h"
#include "main.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>

//  // APRSlib stuff
#include "aprs.h"
#include "aprsConfig.h"
#include "ax25.h"

//  // Include GNSS driver for MAX-M10S
#include "gps_types.h"
#include "max_m10s.h"
#include "ubx.h"
#include "ubx_messages.h"  // for ubx_nav_pvt_s

//  // Include BME Driver
#include "bme68x_driver.h"

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
// Macro defining the prototype for the function that redirects printf output to UART

/* --- Sensor addresses and register definitions --- */
#define LTR329_ADDR (0x29 << 1)  // I2C address of the LTR-329 ambient light sensor (shifted for read/write operations)
#define LTR329_CONTR 0x80        // Control register for configuring the LTR-329 sensor
#define LTR329_MEAS_RATE 0x85    // Register to set the measurement rate and integration time
#define LTR329_DATA_START 0x88   // Register address where ambient light data starts

#define MCP9808_ADDR (0x18 << 1)       // I2C address of the MCP9808 temperature sensor (shifted for read/write operations)
#define MCP9808_REG_AMBIENT_TEMP 0x05  // Register for reading the ambient temperature data

#define MAX_M10S_DEFAULT_ADDR (0x42)  // Default I2C address of the MAX-M10S GPS module (shifted for read/write)
#define sec 20

#define BME68X_ADDR (0x77 << 1)  // 0xEE
#define SEALEVEL_PRESSURE 101325

/* USER CODE BEGIN PV */
volatile uint8_t buttonPressed = 0;  // Flag to indicate if the button has been pressed (1 = pressed, 0 = not pressed)
volatile uint32_t Threshold = 1000;  // ADC threshold value corresponding to 2.3V (calculated using Vin/Vref * (2^n - 1))
volatile uint32_t value_adc = 0;     // Variable to store the ADC conversion result (raw digital value)
volatile float latitude = 0.0f;
volatile float longitude = 0.0f;
volatile float tempVal = 0.0f;
volatile uint16_t lightVal = 0;
char aprsFrame[100];  // APRS Frame array
char latStr[30]; //str for aprs
char lonStr[30]; //str for aprs
char tempStr[16];
char pressureStr[16];
char humidityStr[16];
char gasStr[16];
char altStr[16];

// GPS Static variables
static max_m10s_dev_s gps_dev;
static max_m10s_init_s gps_init;

// APRS input values
uint8_t analogValues[5][5] = {
    "001,",
    "002,",
    "003,",
    "004,",
    "005,"
};
char *digitalValue = "00001100";
char *comment = "hello!";

/* ================================ */
/*          Helper Functions        */
/* ================================ */

/* --- Retarget printf to UART2 --- */
/* Overrides putchar to send characters through UART2, allowing printf debugging */
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, COM_POLL_TIMEOUT);
    return ch;
}


/* ================================ */
/*       LED Blinking Functions     */
/* ================================ */

/* ================================ */
/*       MAX10s GPS Functions       */
/* ================================ */
static const char* print_status(const char* message, gps_status_e status) {
    const char* status_str;
    switch (status) {
        case UBLOX_OK:
            status_str = "OK";
            break;
        case UBLOX_ERROR:
            status_str = "ERROR";
            break;
        case UBLOX_TIMEOUT:
            status_str = "TIMEOUT";
            break;
        case UBLOX_INVALID_PARAM:
            status_str = "INVALID PARAM";
            break;
        case UBLOX_CHECKSUM_ERR:
            status_str = "CHECKSUM ERROR";
            break;
        case UBLOX_I2C_ERROR:
            status_str = "I2C ERROR";
            break;
        default:
            status_str = "UNKNOWN";
            break;
    }
    printf("%s: %s (0x%02X)\r\n", message, status_str, status);

    return status_str;
}

void GPS_ReadOnce(void) {
    gps_status_e status;

    // — send the PVT command —
    status = max_m10s_command(&gps_dev, GPS_CMD_PVT);
    print_status("PVT cmd", status);
    if (status != UBLOX_OK) return;

    HAL_Delay(100);

    // — read & validate —
    status = max_m10s_read(&gps_dev);
    print_status("PVT read", status);
    if (status != UBLOX_OK) return;
    status = max_m10s_validate_response(&gps_dev, GPS_CMD_PVT);
    print_status("PVT validate", status);
    if (status != UBLOX_OK) return;

    // payload starts at buf[6]
    ubx_nav_pvt_s* pvt = (ubx_nav_pvt_s*)&gps_dev.rx_buffer[6];

    printf("PVT fixType=%u, gnssFixOK=%u, validTime=%u\n",
           pvt->fixType,
           pvt->flags.bits.gnssFixOK,
           pvt->valid.bits.validTime);

    if (pvt->fixType < 2 || !pvt->flags.bits.gnssFixOK) {
        printf("  No fix yet – waiting for satellites…\r\n");
        return;
    }

    // — manual parse of lon/lat —
    uint8_t* buf = gps_dev.rx_buffer;
    // Note: payload starts at index 6 of buf.
    int32_t lon_raw = (int32_t)((buf[6 + 24]) |
                                (buf[6 + 25] << 8) |
                                (buf[6 + 26] << 16) |
                                (buf[6 + 27] << 24));
    int32_t lat_raw = (int32_t)((buf[6 + 28]) |
                                (buf[6 + 29] << 8) |
                                (buf[6 + 30] << 16) |
                                (buf[6 + 31] << 24));
    

    // Float printing was giving me trouble, so this is the workaround for that
    int32_t lat_deg = lat_raw / 10000000;
    int32_t lat_rem = abs(lat_raw % 10000000);

    int32_t lon_deg = lon_raw / 10000000;
    int32_t lon_rem = abs(lon_raw % 10000000);

    if (lat_deg > 90 || lat_deg < -90 || lon_deg > 180 || lon_deg < -180) // checking for correct data
        printf("Incorrect data for GPS location");
    else
        // print with integer formatting
       // printf("GPS fix:  Lat=%ld.%07ld°, Lon=%ld.%07ld°\r\n",
         //   lat_deg, lat_rem, lon_deg, lon_rem);
        {   printf("GPS fix:\n");
            sprintf(latStr, "%ld.%02ld", lat_deg, lat_rem);
            sprintf(lonStr, "%ld.%02ld", lon_deg, lon_rem);

}}

// Apparently cold-starts take 30-60s, so we have to wait for the gps module to get a fix
void wait_for_gps(void) {
    ubx_nav_pvt_s* pvt;
    const int max_attempts = 5;
    int attempt = 0;

    do {
         GPS_ReadOnce();   // send NAV-PVT, read & validate
        HAL_Delay(1000);  // pause 1 s between tries

        // point at the PVT payload in the buffer
        pvt = (ubx_nav_pvt_s*)&gps_dev.rx_buffer[6];

        // optional debug:
        printf("  try %d: fixType=%u, gnssFixOK=%u\n",
               attempt + 1,
               pvt->fixType,
               pvt->flags.bits.gnssFixOK);

        attempt++;
    } while (attempt < max_attempts &&
             (pvt->fixType < 2 || !pvt->flags.bits.gnssFixOK));

    if (pvt->fixType < 2 || !pvt->flags.bits.gnssFixOK) {
        printf("GPS fix failed after %d attempts\n", max_attempts);
        return;
    }

}
/* ================================ */
/*       BME68x Sensor Function     */
/* ================================ */

void BME_SensorRead(void) {
    static bme68x_sensor_t bme;               // your “instance” of the sensor
    bme_init(&bme, &hi2c1, &delay_us_timer);  // wire it up to hi2c1 and your delay fn
    

    // Quick status check
    if (bme_check_status(&bme) != BME68X_OK) {
        printf("BME68x init failed!\r\n");
        return;
    }

    // Configure oversampling: T×2, P×16, H×1 (same as the Arduino defaults)
    bme_set_TPH_default(&bme);

    // Set up the heater for a single forced‐mode cycle (300 °C for 100 ms)
    bme_set_heaterprof(&bme, 300, 100);

    // Trigger one forced measurement
    bme_set_opmode(&bme, BME68X_FORCED_MODE);

    // Wait the required duration (driver gives you µs)
    uint32_t wait_us = bme_get_meas_dur(&bme, BME68X_FORCED_MODE);
    HAL_Delay((wait_us + 999) / 1000);  // round up to ms

    // Fetch the data
    uint8_t n_fields = bme_fetch_data(&bme);
    if (n_fields == 0) {
        printf("No new data from BME68x\n");
        return;
    }


    // but ints work
    int32_t rawT = bme.sensor_data.temperature;     // e.g. 2236  => 22.36 °C
    int32_t rawP = bme.sensor_data.pressure;        // e.g. 101325
    int32_t rawH = bme.sensor_data.humidity;        // e.g. 45123 => 45.123 %
    int32_t rawG = bme.sensor_data.gas_resistance;  // e.g. 12000 Ω

    // the whole altitude thing is weird. it works better outside
    // I'll leave the code in here since it's not breaking anything anyways.
    float altitude = 44330.0f * (1.0f - powf((float)rawP / SEALEVEL_PRESSURE, 0.1902949f));
    // round to nearest meter:
    long alt_int = (long)lroundf(altitude);

   // printf("BME68x: T=%ld.%02ld °C, P=%ld Pa, H=%ld.%02ld%%, G=%ld Ω, Alt=%ld m\n",
  //         rawT / 100, abs(rawT % 100),
   //        rawP,
    //       rawH / 1000, abs(rawH % 1000),
     //      rawG,
      //     alt_int);

sprintf(tempStr, "%ld.%02ld°C", rawT / 100, abs(rawT % 100));
sprintf(pressureStr, "%ldPa", rawP);
sprintf(humidityStr, "%ld.%02ld%%", rawH / 1000, abs((rawH % 1000) / 10));  // Only keep 2 decimals
sprintf(gasStr, "%ldΩ", rawG);
sprintf(altStr, "%ldm", alt_int);

}



/* ================================ */
/*       System Initialization      */
/* ================================ */

/* --- Initialize system and peripherals --- */
void INIT() {
    printf("\nInitializing HAL...\n");
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
    printf("\nInitializing I2C...\n");
    MX_I2C1_Init();
    printf("Scanning i2c bus...\n");
    i2c_scan();

    //  --- GPS init ---
    printf("Initializing GPS...\r\n");
    gps_init.hi2c = &hi2c1;
    gps_init.device_address = MAX_M10S_DEFAULT_ADDR;
    gps_init.timeout_ms = 1000;
    gps_init.transmit = HAL_I2C_Master_Transmit;
    gps_init.receive = HAL_I2C_Master_Receive;
    gps_init.delay_blocking = HAL_Delay;

    if (max_m10s_init(&gps_dev, &gps_init) != UBLOX_OK) {
        printf("GPS init failed!\r\n");
    } else {
        print_status("GPS init", UBLOX_OK);
        // 10 Hz PVT rate = 100 ms
        if (max_m10s_config_meas_rate(&gps_dev, 100) == UBLOX_OK) {
            print_status("Set meas rate", UBLOX_OK);
        }
    }

    HAL_Delay(50);
    MX_TIM2_Init();
    if (HAL_I2C_IsDeviceReady(&hi2c1, BME68X_ADDR, 3, HAL_MAX_DELAY) == HAL_OK) {
        printf("Sensor is ready\r\n");
    } else {
        printf("Sensor not responding\r\n");
    }
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
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef* hrtc) {
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
    printf("-----------------------------------------------\n\n");
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

    // Actual voltage value = (ADC_VAL / (2^n - 1)) * Vref
    // Print the current ADC value and the threshold value to UART (debugging output)
    printf("ADC Raw Value: %lu\n", value_adc);
    printf("Threshold:     %lu\n\n", Threshold);

    HAL_Delay(2000);  // Delay to allow some time before next reading (optional for testing)
}

void APRS_CreatePacket(uint8_t *analogValues, char *digitalValue, char *comment,char *latStr,char *lonStr, const char *status_str){
    telemetryInfoFrame *tFrame = initTFrame();
    updateTelemData(tFrame, analogValues,(uint8_t*) digitalValue, comment);
    concatTelemData(tFrame);

    //position frame
    positionFrame *pFrame = initPositionFrame();
    updatePositionData(pFrame, time, latStr, lonStr, comment);
    concatPositionData(pFrame);

    //wrap aprs packet in ax25 packet
    ax25Frame *frame = initFrame(tFrame->tData, tFrame->tDataSize);
    encodedAx25Frame encodedFrame = processFrameVerbose(frame);
    

    //clean up
    freeFrames(tFrame, NULL, NULL, NULL, NULL, &encodedFrame);
    
    //blinky
    HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
   
    HAL_Delay(10000);
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
        // GPS_ReadOnce();
        wait_for_gps();
        printf("back in main(), now calling BME_SensorRead()\r\n");
        BME_SensorRead();
        APRS_CreatePacket(analogValues, &digitalValue, &comment &latStr, &lonStr, status_str);
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

