#pragma once
#include "stm32l4xx_hal.h"
#include "logging.h"

// hal_interface.h

// Forward declarations for generic HAL callbacks using function pointer typedefs
typedef void (*hal_tx_complete_cb)(void* context);
typedef void (*hal_rx_complete_cb)(void* context);
typedef void (*hal_error_cb)(void* context, HAL_StatusTypeDef error);
typedef void (*hal_delay_complete_cb)(void* context);


typedef struct {
    void* context;
    hal_tx_complete_cb tx_complete;
    hal_rx_complete_cb rx_complete;
    hal_error_cb error;
    hal_delay_complete_cb delay_complete;
} hal_callbacks_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    TIM_HandleTypeDef *htim;  // For delay timing
    hal_callbacks_t callbacks;
    struct {
        // I2C operations
        HAL_StatusTypeDef (*transmit_blocking)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                             uint8_t *data, uint16_t size, uint32_t timeout);
        HAL_StatusTypeDef (*receive_blocking)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                            uint8_t *data, uint16_t size, uint32_t timeout);
        HAL_StatusTypeDef (*transmit_it)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                        uint8_t *data, uint16_t size);
        HAL_StatusTypeDef (*receive_it)(I2C_HandleTypeDef *hi2c, uint16_t addr,
                                       uint8_t *data, uint16_t size);
        // Delay operations
        HAL_StatusTypeDef (*delay_it)(TIM_HandleTypeDef *htim, uint32_t delay_ms);
    } ops;
} hal_interface_t;


HAL_StatusTypeDef hal_interface_init(hal_interface_t *hal);
