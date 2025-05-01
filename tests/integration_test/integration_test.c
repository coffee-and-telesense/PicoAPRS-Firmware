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

// GPS Static variables
static max_m10s_dev_s gps_dev;
static max_m10s_init_s gps_init;

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
/*       LTR-329 Sensor Functions   */
/* ================================ */
/* --- LTR-329 initialization --- */
// static void LTR329_Init(void) {
//     uint8_t data[2];
//     data[0] = LTR329_CONTR;
//     data[1] = 0x01;  // Active mode, gain 1x
//     HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

//     data[0] = LTR329_MEAS_RATE;
//     data[1] = 0x02;  // 2Hz integration, 100ms measurement
//     HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, data, 2, HAL_MAX_DELAY);

//     printf("LTR-329 Initialized!\r\n");
// }

// /* --- LTR-329 read ambient light --- */
// static uint16_t LTR329_ReadALS(void) {
//     uint8_t reg = LTR329_DATA_START;
//     uint8_t data[4];
//     HAL_I2C_Master_Transmit(&hi2c1, LTR329_ADDR, &reg, 1, HAL_MAX_DELAY);
//     HAL_I2C_Master_Receive(&hi2c1, LTR329_ADDR, data, 4, HAL_MAX_DELAY);
//     return (uint16_t)((data[1] << 8) | data[0]);
// }

/* ================================ */
/*       MCP9808 Sensor Functions   */
/* ================================ */
// /* --- MCP9808 initialization --- */
// static void MCP9808_Init(void) {
//     printf("MCP9808 Initialized!\r\n");
// }

// /* --- MCP9808 read temperature in Celsius --- */
// static float MCP9808_ReadTemp(void) {
//     uint8_t reg = MCP9808_REG_AMBIENT_TEMP;
//     uint8_t data[2];
//     HAL_I2C_Master_Transmit(&hi2c1, MCP9808_ADDR, &reg, 1, HAL_MAX_DELAY);
//     HAL_I2C_Master_Receive(&hi2c1, MCP9808_ADDR, data, 2, HAL_MAX_DELAY);

//     uint16_t raw = (data[0] << 8) | data[1];
//     float temperature = (raw & 0x0FFF) / 16.0f;
//     if (raw & 0x1000) {  // negative bit
//         temperature -= 256;
//     }
//     return temperature;
// }

/* ================================ */
/*       LED Blinking Functions     */
/* ================================ */

/* ================================ */
/*       MAX10s GPS Functions       */
/* ================================ */
static void print_status(const char* message, gps_status_e status) {
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
}
// === 1) Define a one‐shot GPS read function ===
// void GPS_ReadOnce(void) {
//     gps_status_e status;

//     // 1a) send the PVT command
//     status = max_m10s_command(&gps_dev, GPS_CMD_PVT);
//     print_status("PVT cmd", status);
//     if (status != UBLOX_OK) return;

//     // 1b) small pause to let the module prep its data
//     HAL_Delay(100);

//     // 1c) read the response
//     status = max_m10s_read(&gps_dev);
//     print_status("PVT read", status);
//     if (status != UBLOX_OK) return;

//     // 1d) validate it
//     status = max_m10s_validate_response(&gps_dev, GPS_CMD_PVT);
//     print_status("PVT validate", status);
//     if (status != UBLOX_OK) return;

//     // 1e) parse UBX‐PVT into a struct
//     gps_pvt_data_t pvt;
//     if (ubx_parse_gps_pvt(gps_dev.rx_buffer, gps_dev.rx_size, &pvt) == UBLOX_OK) {
//         float lat = pvt.lat / 1e7f;
//         float lon = pvt.lon / 1e7f;
//         printf("↳ GPS fix:  Lat=%.7f°, Lon=%.7f°\r\n", lat, lon);
//     } else {
//         printf("↳ PVT parse error\r\n");
//     }
// }
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
    // float lon = lon_raw / 1e7f;
    // float lat = lat_raw / 1e7f;

    // printf("GPS fix:  Lat=%.7f°, Lon=%.7f°\r\n", lat, lon);
    // split into degrees + fractional part

    // Float printing was giving me trouble, so this is the workaround for that
    int32_t lat_deg = lat_raw / 10000000;
    int32_t lat_rem = abs(lat_raw % 10000000);

    int32_t lon_deg = lon_raw / 10000000;
    int32_t lon_rem = abs(lon_raw % 10000000);

    // print with integer formatting
    printf("GPS fix:  Lat=%ld.%07ld°, Lon=%ld.%07ld°\r\n",
           lat_deg, lat_rem, lon_deg, lon_rem);
}

// Apparently cold-starts take 30-60s, so we have to wait for the gps module to get a fix
void wait_for_gps_fix(void) {
    ubx_nav_pvt_s* pvt;

    // keep asking for a PVT until we have at least a 2D fix
    do {
        GPS_ReadOnce();   // send NAV-PVT, read & validate
        HAL_Delay(1000);  // pause 1 s between tries

        // point at the PVT payload in the buffer
        pvt = (ubx_nav_pvt_s*)&gps_dev.rx_buffer[6];

        // optional: print out fixType each time to watch progress
        printf("  fixType=%u, gnssFixOK=%u\n",
               pvt->fixType,
               pvt->flags.bits.gnssFixOK);

    } while (pvt->fixType < 2 || !pvt->flags.bits.gnssFixOK);

    // // at this point we have a fix, so lat/lon are valid
    // int32_t lat_raw = pvt->lat, lon_raw = pvt->lon;
    // int32_t lat_deg = lat_raw / 10000000, lat_rem = abs(lat_raw % 10000000);
    // int32_t lon_deg = lon_raw / 10000000, lon_rem = abs(lon_raw % 10000000);
    // printf("GPS fix: Lat=%ld.%07ld°, Lon=%ld.%07ld°\r\n",
    //        lat_deg, lat_rem,
    //        lon_deg, lon_rem);
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

    // it still doesn't print floats
    //  float T = bme.sensor_data.temperature;     // °C
    //  float P = bme.sensor_data.pressure;        // Pa
    //  float H = bme.sensor_data.humidity;        // %RH (or %RH×1000—check your API)
    //  float G = bme.sensor_data.gas_resistance;  // Ω

    // // If humidity is in ‰ (x1000), divide by 1000.0f to get percent:
    // printf("BME68x: T=%.2f °C, P=%.0f Pa, H=%.3f %%, G=%.0f Ω\r\n",
    //        T,
    //        P,
    //        H / 1000.0f,
    //        G);

    // but ints work
    int32_t rawT = bme.sensor_data.temperature;     // e.g. 2236  => 22.36 °C
    int32_t rawP = bme.sensor_data.pressure;        // e.g. 101325
    int32_t rawH = bme.sensor_data.humidity;        // e.g. 45123 => 45.123 %
    int32_t rawG = bme.sensor_data.gas_resistance;  // e.g. 12000 Ω

    //the whole altitude thing is weird. it works better outside
    //I'll leave the code in here since it's not breaking anything anyways.
    float altitude = 44330.0f * (1.0f - powf((float)rawP / SEALEVEL_PRESSURE, 0.1902949f));
    // round to nearest meter:
    long alt_int = (long)lroundf(altitude);

    printf("BME68x: T=%ld.%02ld°C, P=%ldPa, H=%ld.%03ld%%, G=%ldΩ, Alt=%ldm\n",
           rawT / 100, abs(rawT % 100),
           rawP,
           rawH / 1000, abs(rawH % 1000),
           rawG,
           alt_int);
}

// void BME_SensorRead(void) {
//     // Create bme interface struct and initialize it
//     bme68x_sensor_t bme;
//     bme_init(&bme, &hi2c1, &delay_us_timer);

//     // Check status, should be 0 for OK
//     int bme_status = bme_check_status(&bme);
//     {
//         if (bme_status == BME68X_ERROR) {
//             printf("Sensor error:" + bme_status);
//             return BME68X_ERROR;
//         } else if (bme_status == BME68X_WARNING) {
//             printf("Sensor Warning:" + bme_status);
//         }
//     }
//     // Set temp, pressure, humidity oversampling configuration
//     // Trying with defaults
//     bme_set_TPH_default(&bme);
//     // Set the heater configuration to 300 deg C for 100ms for Forced mode
//     bme_set_heaterprof(&bme, 300, 100);

//     // Read sensor id
//     uint8_t sensor_id;
//     bme_read(0xD0, &sensor_id, 4, &hi2c1);
//     debug_print("Received sensor ID: 0x%X\r\n", sensor_id);

//     // Set to forced mode, which takes a single sample and returns to sleep mode
//     bme_set_opmode(&bme, BME68X_FORCED_MODE);
//     /** @todo: May adjust the specific timing function called here, but it should be based on bme_get_meas_dur */
//     delay_us_timer(bme_get_meas_dur(&bme, BME68X_SLEEP_MODE), &hi2c1);
//     // Fetch data
//     int fetch_success = bme_fetch_data(&bme);
//     if (fetch_success) {
//         // Print raw values
//         debug_print("Raw Temperature     : %d\n", bme.sensor_data.temperature);
//         debug_print("Raw Pressure        : %d\n", bme.sensor_data.pressure);
//         debug_print("Raw Humidity        : %d\n", bme.sensor_data.humidity);
//         debug_print("Raw Gas Resistance  : %d\n", bme.sensor_data.gas_resistance);

//         // Convert and print the processed values
//         // Temperature: Divide by 100 to get the value in °C with 2 decimal places
//         debug_print("Temperature         : %d.%02d°C\n",
//                     (int)(bme.sensor_data.temperature / 100.0f),      // Integer part
//                     (int)fmod(bme.sensor_data.temperature, 100.0f));  // Fractional part (2 decimal places)

//         // Pressure: Divide by 100 to get the value in Pa
//         debug_print("Pressure            : %d Pa\n",
//                     bme.sensor_data.pressure);

//         // Humidity: Divide by 1000 to get the value in % with 3 decimal places
//         debug_print("Humidity            : %d.%03d%%\n",
//                     (int)(bme.sensor_data.humidity / 1000.0f),      // Integer part
//                     (int)fmod(bme.sensor_data.humidity, 1000.0f));  // Fractional part (3 decimal places)

//         // Gas Resistance: Convert to kΩ and display with 3 decimal places
//         debug_print("Gas Resistance      : %d.%03d kΩ\n",
//                     (int)(bme.sensor_data.gas_resistance / 1000.0f),      // Integer part (kΩ)
//                     (int)fmod(bme.sensor_data.gas_resistance, 1000.0f));  // Fractional part (milliΩ)

//         // Print the status in hexadecimal
//         debug_print("Status              : 0x%X\n", bme.sensor_data.status);

//         debug_print("\n---------------------------------------\n");
//     }

//     // The "blink" code is a simple verification of program execution,
//     // separate from the BME68x sensor testing above
//     HAL_GPIO_TogglePin(UserLED_GPIO_Port, UserLED_Pin);
//     HAL_Delay(1000);
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
        // GPS_ReadOnce();
        wait_for_gps_fix();
        printf("back in main(), now calling BME_SensorRead()\r\n");
        BME_SensorRead();

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
