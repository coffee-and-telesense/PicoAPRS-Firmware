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

#ifndef BMV080_EXAMPLE_H_
#define BMV080_EXAMPLE_H_


#ifdef __cplusplus  
extern "C" {
#endif /* __cplusplus */

#include "bmv080_defs.h"
#include "bmv080.h"

// NOTE: BMV080_I2C_ADDRESS originally from combridge.c
/* Private define ------------------------------------------------------------*/
/* BMV080 I2C address
 * note that the BMV080 pins are connected such that I2C Address Bit 0 = 0 and I2C Address Bit 1 = 0
 */
// TODO: Determine which address is used in which contexts
// #define BMV080_I2C_ADDRESS 0x54
#define BMV080_I2C_ADDRESS 0x57

/* Helper macro for updating the final status */
#define UPDATE_OPERATION_STATUS(current_status, final_status) (final_status = ((final_status) != 0) ? (final_status) : (current_status))

/* Helper macro for error reporting and handling */
#define CHECK_ERROR(current_status, final_status, error_message, error_handling)                                                       \
                                    if (current_status != E_BMV080_OK)                                                                 \
                                    {                                                                                                  \
                                        (void)print("%s failed with BMV080 status %d\r\n", error_message, ((int32_t)current_status));  \
                                        UPDATE_OPERATION_STATUS(current_status, final_status);                                         \
                                        goto error_handling;                                                                           \
                                    }                                                                          

/* Define the delay time in milliseconds for PRINT_AND_DELAY */
#define PRINT_DELAY_TIME_MS 10

/* Helper macro for printing and delaying */                        
#define PRINT_AND_DELAY(f_, ...) (void)print((f_), ##__VA_ARGS__); \
                                 (void)delay_ms(PRINT_DELAY_TIME_MS); \

/* Function pointer to facilitate printing output / status information to the console */
typedef int (*print_function_t)(const char *const _format, ...);

/* Function pointer to enable / disable external interrupt connected to IRQ line of BMV080 sensor*/
typedef void (*enable_ext_interrupt_function_t)(bool);

/*!
* @brief Service routine can be called from external interrupt handler to service the BMV080 sensor.
*/
void bmv080_service_routine(void);

/*!
* @brief Custom function for consuming sensor readings based on implementing bmv080_callback_data_ready_t.
*/
void use_sensor_output(bmv080_output_t bmv080_output, void* callback_parameters);

/*!
* @brief Example usage of all BMV080 APIs in the expected sequence.
*
* @param[in] handle :  Unique handle for a serial communication endpoint.
* @param[in] read : Function pointer for reading from an endpoint.
* @param[in] write : Function pointer for writing to an endpoint.
* @param[in] delay_ms : Function pointer for executing a delay in milliseconds.
* @param[in] get_tick_ms : Function pointer for getting a tick value in milliseconds.
* @param[in] print : Function pointer for printing to the console.
* @param[in] enable_ext_interrupt : Function pointer for enabling and disabling the external interrupt line
*                                   connected to th IRQ 1 line from BMV080.
*
* @return E_BMV080_OK if successful, otherwise the return value is a BMV080 status code.
*/
bmv080_status_code_t bmv080(const bmv080_sercom_handle_t sercom_handle, 
                            const bmv080_callback_read_t read, 
                            const bmv080_callback_write_t write, 
                            const bmv080_callback_delay_t delay_ms, 
                            const bmv080_callback_tick_t get_tick_ms, 
                            const print_function_t print,
                            const enable_ext_interrupt_function_t enable_ext_interrupt);

// From combridge.c in Bosch STM32 example for Cortex M4
/* I2C read and write functions */
int8_t combridge_i2c_read_16bit(bmv080_sercom_handle_t handle, uint16_t header, uint16_t *payload, uint16_t payload_length);
int8_t combridge_i2c_write_16bit(bmv080_sercom_handle_t handle, uint16_t header, const uint16_t *payload, uint16_t payload_length);

/* Delay function */
int8_t combridge_delay(uint32_t period);

#ifdef __cplusplus  
}
#endif /* __cplusplus */

#endif /* BMV080_EXAMPLE_H_ */
