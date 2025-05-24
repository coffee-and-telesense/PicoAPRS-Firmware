/**
 * Copyright (C) Bosch Sensortec GmbH. All Rights Reserved. Confidential.
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet. Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchaser's own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 * @file bmv080_example.c
 *
 * @brief Example showing the usage of the BMV080 sensor driver APIs.
 *
 *
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "bmv080_example.h"

/*********************************************************************************************************************
* Variable declarations
*********************************************************************************************************************/
volatile uint32_t data_ready_callback_count = 0;

/* A unique handle is used to address a BMV080 sensor unit */
static bmv080_handle_t bmv080_handle = NULL;

/* handle for print function to be used in interrupt service routine */
static print_function_t print_handle = NULL; 

/*********************************************************************************************************************
* Function definitions
*********************************************************************************************************************/
/* Custom function for consuming sensor readings 
* bmv080_serve_interrupt() is called by the application to read new sensor readings from the BMV080 sensor unit.
* The application can call bmv080_service_routine() in regular intervals (at least once a second) from a loop, in a timer interrupt service routine
* or use the BMV080 sensor unit's hardware interrupt as trigger. 
* The optional callback_parameters can be used by the application as a wild card.
* In this example, the callback_parameters argument is used to count how often the callback function use_sensor_output was called. 
* The callback function reports sensor output and is called once every second based on the sensor's actual measurement time.
*/
void bmv080_service_routine(void)
{       
        if ( (bmv080_handle != NULL) && (print_handle != NULL))
        {
            /* The interrupt is served by the BMV080 sensor driver */
            bmv080_status_code_t bmv080_current_status = bmv080_serve_interrupt(bmv080_handle, (bmv080_callback_data_ready_t)use_sensor_output, (void*)print_handle);
            if (bmv080_current_status != E_BMV080_OK)
            {
                (void)print_handle("Fetching measurement data failed with BMV080 status %d\r\n", (int32_t)bmv080_current_status);
            }
        }
}

/* Custom function for consuming sensor readings */
void use_sensor_output(bmv080_output_t bmv080_output, void* callback_parameters)
{
    data_ready_callback_count++;
    print_function_t print = (print_function_t)callback_parameters;
    
    (void)print("Runtime: %.2f s, PM2.5: %.0f ug/m^3, PM1: %.0f ug/m^3, PM10: %.0f ug/m^3, "
                    "obstructed: %s, outside measurement range: %s\r\n",
        bmv080_output.runtime_in_sec, 
        bmv080_output.pm2_5_mass_concentration, 
        bmv080_output.pm1_mass_concentration, 
        bmv080_output.pm10_mass_concentration,
        (bmv080_output.is_obstructed ? "yes" : "no"), 
        (bmv080_output.is_outside_measurement_range ? "yes" : "no"));
}

/* Example usage of all BMV080 APIs in the expected sequence */
bmv080_status_code_t bmv080(const bmv080_sercom_handle_t sercom_handle, 
                            const bmv080_callback_read_t read, 
                            const bmv080_callback_write_t write, 
                            const bmv080_callback_delay_t delay_ms, 
                            const bmv080_callback_tick_t get_tick_ms, 
                            const print_function_t print,
                            const enable_ext_interrupt_function_t enable_ext_interrupt)
{
    /* All functions of the BMV080 sensor driver return a status code */
    bmv080_status_code_t bmv080_current_status = E_BMV080_OK;

    /* The return value of this main function will be overwritten by the first not-okay status */
    bmv080_status_code_t bmv080_final_status = bmv080_current_status;

    /* init print_handle to be used in interrupt service routine */
    print_handle = print;

    /*********************************************************************************************************************
    * Getting sensor driver version
    *********************************************************************************************************************/
    uint16_t major = 0;
    uint16_t minor = 0;
    uint16_t patch = 0;
    char git_hash[12];
    int32_t commits_ahead = 0;

    bmv080_current_status = bmv080_get_driver_version(&major, &minor, &patch, git_hash, &commits_ahead);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting BMV080 sensor driver version", exit_main);

    PRINT_AND_DELAY("BMV080 sensor driver version: %d.%d.%d.%s.%ld\r\n", major, minor, patch, git_hash, commits_ahead);

    /*********************************************************************************************************************
    * Opening (interaction) with a sensor unit
    *********************************************************************************************************************/
    bmv080_current_status = bmv080_open(
        &bmv080_handle, sercom_handle,
        (const bmv080_callback_read_t)read,
        (const bmv080_callback_write_t)write,
        (const bmv080_callback_delay_t)delay_ms);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Opening BMV080 sensor unit", exit_main);

    /*********************************************************************************************************************
    * Resetting a sensor unit
    *********************************************************************************************************************/
    bmv080_current_status = bmv080_reset(bmv080_handle);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Resetting BMV080 sensor unit", close_sensor);

    /*********************************************************************************************************************
    * Getting the ID of a sensor unit
    *********************************************************************************************************************/
    char id[13];
    memset(id, 0x00, 13);
    bmv080_current_status = bmv080_get_sensor_id(bmv080_handle, id);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the sensor ID", close_sensor);

    PRINT_AND_DELAY("Sensor ID: %s\r\n", id);
    PRINT_AND_DELAY("\r\n");

    /*********************************************************************************************************************
    * Getting (default) configuration parameters
    *********************************************************************************************************************/
    PRINT_AND_DELAY("Get parameters\r\n");

    /* Get default parameter "integration_time" */
    float integration_time = 0.0f;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "integration_time", (void*)&integration_time);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"integration_time\"", close_sensor);

    PRINT_AND_DELAY("Default integration_time: %.1f s\r\n", integration_time);

    /* Get default parameter "measurement_algorithm" */
    bmv080_measurement_algorithm_t measurement_algorithm = E_BMV080_MEASUREMENT_ALGORITHM_BALANCED;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "measurement_algorithm", (void*)&measurement_algorithm);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"measurement_algorithm\"", close_sensor);

    PRINT_AND_DELAY("Default measurement_algorithm: %d\r\n", measurement_algorithm);

    /* Get default parameter "do_obstruction_detection" */
    bool do_obstruction_detection = false;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "do_obstruction_detection", (void*)&do_obstruction_detection);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"do_obstruction_detection\"", close_sensor);

    PRINT_AND_DELAY("Default do_obstruction_detection: %s\r\n", (do_obstruction_detection ? "true" : "false"));

    /* Get default parameter "do_vibration_filtering" */
    bool do_vibration_filtering = false;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "do_vibration_filtering", (void*)&do_vibration_filtering);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"do_vibration_filtering\"", close_sensor);

    PRINT_AND_DELAY("Default do_vibration_filtering: %s\r\n", (do_vibration_filtering ? "true" : "false"));

    /* The file-logging related parameters (error_logging, mdf_logging, pm_logging and path) are available only on platforms: Windows (x86/x64) and RaspberryPi */
#if defined(_MSC_VER) || defined(__gnu_linux__)
    /* Get default parameter "error_logging" */
    bool erf_logging = false;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "error_logging", (void*)&erf_logging);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"error_logging\"", close_sensor);

    PRINT_AND_DELAY("Default error_logging: %s\r\n", (erf_logging ? "true" : "false"));

    /* Get default parameter "meta_data_logging" */
    bool mdf_logging = false;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "meta_data_logging", (void*)&mdf_logging);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"meta_data_logging\"", close_sensor);

    PRINT_AND_DELAY("Default meta_data_logging: %s\r\n", (mdf_logging ? "true" : "false"));

    /* Get default parameter "pm_logging" */
    /* This is the PM2.5 output logging */
    bool pm_logging = false;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "pm_logging", (void*)&pm_logging);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"pm_logging\"", close_sensor);

    PRINT_AND_DELAY("Default pm_logging: %s\r\n", (pm_logging ? "true" : "false"));

    /* Get default parameter "path"
     * The maximum size of the returned parameter is 256 characters */
    char path[256] = { '\0' };
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "path", (void*)path);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"path\"", close_sensor);

    PRINT_AND_DELAY("Default path: \"%s\"\r\n", path);
#endif
    PRINT_AND_DELAY("\r\n");

    /*********************************************************************************************************************
    * Setting (custom) configuration parameters
    *********************************************************************************************************************/
    PRINT_AND_DELAY("Set parameters\r\n");

    /* Set custom parameter "integration_time" */
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "integration_time", (void*)&integration_time);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"integration_time\"", close_sensor);

    PRINT_AND_DELAY("Customized integration_time: %.1f s\r\n", integration_time);

    /* Set custom parameter "measurement_algorithm" */
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "measurement_algorithm", (void*)&measurement_algorithm);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"measurement_algorithm\"", close_sensor);

    PRINT_AND_DELAY("Customized measurement_algorithm: %d\r\n", measurement_algorithm);

    /* Set custom parameter "do_obstruction_detection" */
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "do_obstruction_detection", (void*)&do_obstruction_detection);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"do_obstruction_detection\"", close_sensor);

    PRINT_AND_DELAY("Customized do_obstruction_detection: %s\r\n", (do_obstruction_detection ? "true" : "false"));

    /* Set custom parameter "do_vibration_filtering" */
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "do_vibration_filtering", (void*)&do_vibration_filtering);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"do_vibration_filtering\"", close_sensor);
    
    PRINT_AND_DELAY("Customized do_vibration_filtering: %s\r\n", (do_vibration_filtering ? "true" : "false"));

    /* The file-logging related parameters (error_logging, mdf_logging, pm_logging and path) are available only on platforms: Windows (x86/x64) and RaspberryPi */
#if defined(_MSC_VER) || defined(__gnu_linux__)
    /* Set custom parameter "error_logging" */
    erf_logging = true;
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "error_logging", (void*)&erf_logging);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"error_logging\"", close_sensor);
    
    PRINT_AND_DELAY("Customized error_logging : %s\r\n", (erf_logging ? "true" : "false"));

    /* Set custom parameter "meta_data_logging" */
    mdf_logging = true;
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "meta_data_logging", (void*)&mdf_logging);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"meta_data_logging\"", close_sensor);
    
    PRINT_AND_DELAY("Customized meta_data_logging: %s\r\n", (mdf_logging ? "true" : "false"));

    /* Set custom parameter "pm_logging" */
    /* This is the PM2.5 output logging */
    pm_logging = true;
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "pm_logging", (void*)&pm_logging);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"pm_logging\"", close_sensor);
    
    PRINT_AND_DELAY("Customized pm_logging: %s\r\n", (pm_logging ? "true" : "false"));

    /* Set custom parameter "path" */
    char* custom_path = "./logs/";
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "path", (void*)custom_path);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"path\"", close_sensor);
    
    PRINT_AND_DELAY("Customized path: \"%s\"\r\n", custom_path);
#endif
    PRINT_AND_DELAY("\r\n");

    /*********************************************************************************************************************
    * Running a particle measurement in continuous mode
    *********************************************************************************************************************/
    if(enable_ext_interrupt != NULL)
    {
#ifdef USE_EXTERNAL_INTERRUPT
        enable_ext_interrupt((bool)true); // Use of hardware interrupt of the BMV080 sensor unit can be used as trigger.
#else
        enable_ext_interrupt((bool)false); // Use of hardware interrupt of the BMV080 sensor unit is not used as trigger.
#endif
    }

    /* Start particle measurement in continuous mode */
    bmv080_current_status = bmv080_start_continuous_measurement(bmv080_handle);
    /* Check the software & hardware compatibility */
    if (bmv080_current_status == E_BMV080_ERROR_INCOMPATIBLE_SENSOR_HW) 
    {  
        PRINT_AND_DELAY("+--------------------------------------------------------------------------------+\r\n");
        PRINT_AND_DELAY("| The BMV080-SDK is not compatible with the connected BMV080 sensor.             |\r\n");
        PRINT_AND_DELAY("| Please get in contact with your BMV080 representative for sample availability. |\r\n");
        PRINT_AND_DELAY("+--------------------------------------------------------------------------------+\r\n");
        PRINT_AND_DELAY("\r\n");
    }
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Starting measurement in continuous mode", close_sensor);

    /* Execute a continuous particle measurement over a duration of "sensor_measurement_duration_seconds" seconds.
     * In this example, bmv080_serve_interrupt is called repeatedly with a suitable delay after each call.
     * This behavior can differ depending on the application.
     * Alternatives are regular intervals, e.g. at least each second, or use the BMV080 sensor unit's hardware interrupt as trigger. 
     * The optional callback_parameters can be used by the application as a wild card.
     * In this example, the callback_parameters argument is used to count how often the callback function use_sensor_output was called. 
     * The callback function reports sensor output and is called once every second based on the sensor's actual measurement time.
     */
    uint32_t sensor_measurement_duration_seconds = 60;
    data_ready_callback_count = 0;

    PRINT_AND_DELAY("Particle measurement started in continuous mode for %d seconds \r\n", sensor_measurement_duration_seconds);

    while (data_ready_callback_count < sensor_measurement_duration_seconds)
    {
        #if !defined(USE_EXTERNAL_INTERRUPT)
           /* The BMV080 sensor unit's hardware interrupt is not used, call the  bmv080_service_routine 
            * repeatedly with a suitable delay after each call. If the BMV080 sensor unit's hardware interrupt
            * is used, bmv080_serve_interrupt is invoked bmv080_service_routine.
            */
            bmv080_service_routine();
            if (bmv080_current_status != E_BMV080_OK)
            {
                (void)print("Fetching measurement data failed with BMV080 status %d\r\n", (int32_t)bmv080_current_status);
            }

            // The sensor is polled at intervals of 100ms to make sure that even in high particle concentrations no data is lost
            (void)delay_ms(100); 
        #else
            (void)delay_ms(sensor_measurement_duration_seconds * 1000);
            break; // 60s measurement done
        #endif
    }

    /* Stop particle measurement */
    bmv080_current_status = bmv080_stop_measurement(bmv080_handle);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Stopping measurement", close_sensor);

    PRINT_AND_DELAY("Particle measurement stopped\r\n");
    PRINT_AND_DELAY("\r\n");

    /*********************************************************************************************************************
    * Running a particle measurement in duty cycling mode
    *********************************************************************************************************************/
    /* Get default parameter "duty_cycling_period" */
    uint16_t duty_cycling_period = 0;
    bmv080_current_status = bmv080_get_parameter(bmv080_handle, "duty_cycling_period", (void*)&duty_cycling_period);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Getting the parameter \"duty_cycling_period\"", close_sensor);

    PRINT_AND_DELAY("Default duty_cycling_period: %d s\r\n", duty_cycling_period);

    /* Set custom parameter "duty_cycling_period" */
    duty_cycling_period = 20;
    bmv080_current_status = bmv080_set_parameter(bmv080_handle, "duty_cycling_period", (void*)&duty_cycling_period);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Setting the parameter \"duty_cycling_period\"", close_sensor);

    PRINT_AND_DELAY("Customized duty_cycling_period: %d s\r\n", duty_cycling_period);
    
    #ifdef USE_EXTERNAL_INTERRUPT
        if(enable_ext_interrupt == NULL)
        {
            (void)print("Function enable_ext_interrupt() routine is not available.\r\n");
            goto exit_main;
        }    
        enable_ext_interrupt(false); // The hardware interrupt of the BMV080 sensor unit cannot be used as trigger in duty cycling mode.
    #endif

    /* Start particle measurement in duty cycling mode */
    bmv080_duty_cycling_mode_t duty_cycling_mode = E_BMV080_DUTY_CYCLING_MODE_0;
    bmv080_current_status = bmv080_start_duty_cycling_measurement(bmv080_handle, (const bmv080_callback_tick_t)get_tick_ms, duty_cycling_mode);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Starting measurement in duty cycling mode", close_sensor);

    PRINT_AND_DELAY("\r\n");
    PRINT_AND_DELAY("Particle measurement started in duty cycling mode for %d seconds \r\n", sensor_measurement_duration_seconds);

    /* Execute a duty cycling particle measurement over a duration of "sensor_measurement_duration_seconds" seconds.
     * In duty cycling mode, bmv080_serve_interrupt must be called by the application in regular intervalls at least once every second.
     * The application can call bmv080_serve_interrupt in a loop or in a timer interrupt service routine. 
     * The optional callback_parameters can be used by the application as a wild card.
     * In this example, the callback_parameters argument is used to count how often the callback function use_sensor_output was called.
     * The callback function reports sensor output and is called once every duty cycling period at the end of each integration time, 
     * based on sensor's actual measurement time, e.g. for the applied settings this could be at seconds 10, 30, 50,...
     */
    data_ready_callback_count = 0;
    while ((data_ready_callback_count * duty_cycling_period) < sensor_measurement_duration_seconds)
    {
        bmv080_current_status = bmv080_serve_interrupt(bmv080_handle, (bmv080_callback_data_ready_t)use_sensor_output, (void*)print);
        if (bmv080_current_status != E_BMV080_OK)
        {
            (void)print("Fetching measurement data failed with BMV080 status %d\r\n", (int32_t)bmv080_current_status);
        }

        (void)delay_ms(100);
    }

    /* Stop particle measurement */
    bmv080_current_status = bmv080_stop_measurement(bmv080_handle);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Stopping measurement", close_sensor);
    PRINT_AND_DELAY("Particle measurement stopped\r\n");

close_sensor:
    /* Close the sensor unit */
    bmv080_current_status = bmv080_close(&bmv080_handle);
    CHECK_ERROR(bmv080_current_status, bmv080_final_status, "Closing BMV080 sensor unit", exit_main);

exit_main:
    return bmv080_final_status;
}

// From combridge.c in Bosch STM32 example for Cortex M4
#include "stm32u0xx_hal.h"

int8_t combridge_i2c_read_16bit(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length)
{
    HAL_StatusTypeDef return_value;
    I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)handle;

    /* verify function parameters */
    if ((handle == NULL) || (payload == NULL))
    {
        return HAL_ERROR;
    }

    if ((payload_length == 0))
    {
        return HAL_ERROR;
    }

    /* 7 bit I2C address + most significant bit of the header passed from the sensor driver which represents the R/W bit (R = high) */
    uint16_t device_address = (BMV080_I2C_ADDRESS << 1) | (header >> 15);
    /* 16 bit header left shifted, since the R/W bit has already been passed */
    uint16_t header_adjusted = header << 1;

    return_value = HAL_I2C_Mem_Read(i2c_handle, device_address, header_adjusted, I2C_MEMADD_SIZE_16BIT, (uint8_t *)payload, payload_length * 2, HAL_MAX_DELAY);

    /* Conversion of payload from big endian to little endian */
    for (int payload_idx = 0; payload_idx < payload_length; payload_idx++)
    {
        uint16_t swapped_word = ((payload[payload_idx] << 8) | (payload[payload_idx] >> 8)) & 0xffff;
        payload[payload_idx] = swapped_word;
    }

    return (int8_t)return_value;
}

int8_t combridge_i2c_write_16bit(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length)
{
    HAL_StatusTypeDef return_value;
    I2C_HandleTypeDef *i2c_handle = (I2C_HandleTypeDef *)handle;

    /* verify function parameters */
    if ((handle == NULL) || (payload == NULL))
    {
        return HAL_ERROR;
    }

    if ((payload_length == 0))
    {
        return HAL_ERROR;
    }

    /* 7 bit I2C address + most significant bit of the header passed from the sensor driver which represents the R/W bit (R = high) */
    uint16_t device_address = (BMV080_I2C_ADDRESS << 1) | (header >> 15);
    /* 16 bit header left shifted, since the R/W bit has already been passed */
    uint16_t header_adjusted = header << 1;

    /* Conversion of payload from little endian to big endian (dynamic allocation is used) */
    uint16_t *payload_swapped = (uint16_t *)calloc(payload_length, sizeof(uint16_t));
    for (int payload_idx = 0; payload_idx < payload_length; payload_idx++)
    {
        payload_swapped[payload_idx] = ((payload[payload_idx] << 8) | (payload[payload_idx] >> 8)) & 0xffff;
    }

    return_value = HAL_I2C_Mem_Write(i2c_handle, device_address, header_adjusted, I2C_MEMADD_SIZE_16BIT, (uint8_t *)payload_swapped, payload_length * 2, HAL_MAX_DELAY);

    free(payload_swapped);

    return (int8_t)return_value;
}

int8_t combridge_delay(uint32_t period)
{
    HAL_Delay(period);
    return HAL_OK;
}
