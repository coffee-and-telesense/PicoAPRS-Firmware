#include "max_m10s.h"

/**
 * @brief Inserts wait states for I2C to complete, this allows for blocking type behavior
 *         even if driver is compiled for non-blocking operations.
 * @param dev 
 * @param hi2c 
 * @return gps_status_e 
 */

/* Function to initialize the GPS device */
gps_status_e max_m10s_init(max_m10s_dev_s *dev, const max_m10s_init_s *init)
{
    gps_status_e result = UBLOX_OK; 
    uint16_t msg_size; // Size of the config message to be sent
    // Validate parameters
    if (dev == NULL || init == NULL || init->hi2c == NULL) {
        return UBLOX_INVALID_PARAM; 
    }
    // Check that all required function pointers are provided
    if (init->transmit == NULL || 
        init->receive == NULL ||
        init->delay_blocking == NULL) {
        return UBLOX_INVALID_PARAM;
    }
    
    // Copy the initialization structure
    dev->configs = *init;
    // Adjust address to 7-bit format
    dev->configs.device_address = (init->device_address << 1);
    
    // Initialize device state
    dev->initialized = 0;
    dev->tx_size = 0;
    dev->rx_size = 0;
    
    // Initialize UBX protocol for I2C communication
    msg_size = ubx_prepare_config_cmd_u8(dev->tx_buffer, UBX_CFG_I2C_UBX_ENABLE, (uint8_t) 1);
    dev->tx_size = msg_size;

    HAL_StatusTypeDef hal_status;

    hal_status = GPS_TRANSMIT(dev, dev->configs.device_address, dev->tx_buffer, dev->tx_size);

    if(hal_status != HAL_OK) {
        return UBLOX_ERROR; // Error status code
    }
    // Wait for I2C to complete
    I2C_WAIT(dev);
    
    //Ensure delay time is completed, this requires one second delay, I'm not sure why. 
    dev->configs.delay_blocking(1000); // 1 s delay
    // Step 3: Check for ACK response
    hal_status = GPS_RECEIVE(dev, dev->configs.device_address, dev->rx_buffer, 10); // Expecting 10 bytes for ACK response
    
    if (hal_status != HAL_OK) {
        return UBLOX_I2C_ERROR; // Error status code
    }
    // Wait for I2C to complete
    I2C_WAIT(dev);

    // Validate ACK response
    result = ubx_validate_ack(dev->rx_buffer, 10, UBX_CLASS_CFG, UBX_CFG_VALSET);
    if (result != UBLOX_OK) {
        return result; // Error status code
    }

    // Disable NMEA output
    msg_size = ubx_prepare_config_cmd_u8(dev->tx_buffer, UBX_CFG_I2C_NMEA_DISABLE, (uint8_t) 0);
    dev->tx_size = msg_size;

    hal_status = GPS_TRANSMIT(dev, dev->configs.device_address, dev->tx_buffer, dev->tx_size);

    if(hal_status != HAL_OK) {
        return UBLOX_ERROR; // Error status code
    }

    // Wait for I2C to complete
    I2C_WAIT(dev);

    
    //Ensure delay time is completed
    dev->configs.delay_blocking(1000); // 1 s delay
    //Check for ACK response
    hal_status = GPS_RECEIVE(dev, dev->configs.device_address, dev->rx_buffer, 10); // Expecting 10 bytes for ACK response
    if (hal_status != HAL_OK) {
        return UBLOX_I2C_ERROR; // Error status code
    }
    // Wait for I2C to complete
    I2C_WAIT(dev);

    // Validate ACK response
    result = ubx_validate_ack(dev->rx_buffer, 10, UBX_CLASS_CFG, UBX_CFG_VALSET);
    if (result != UBLOX_OK) {
        return result; // Error status code
    }
    dev->initialized = true;
    return UBLOX_OK; // Success status code
}

/**
 * @brief Generic non-blocking command interface for MAX-M10S GPS
 * @param dev Pointer to MAX-M10S device structure
 * @param cmd_type Type of command to execute (from enum gps_cmd_type_e)
 * @return UBLOX_OK if request initiated successfully; else error code
 */
gps_status_e max_m10s_command(max_m10s_dev_s *dev, gps_cmd_type_e cmd_type) {
    // Validate parameters
    if (dev == NULL || !dev->initialized) {
        return UBLOX_INVALID_PARAM;
    }
    // Prepare command based on cmd_type
    uint16_t msg_size = 0;

    switch (cmd_type) {
        case GPS_CMD_PVT:
            msg_size = ubx_prepare_command(dev->tx_buffer, UBX_CLASS_NAV, UBX_NAV_PVT);
            break;
        // Add more cases for other commands
        default:
            return UBLOX_INVALID_PARAM;
        }

    if (msg_size == 0) {
        return UBLOX_ERROR;
    }

    dev->tx_size = msg_size;
    dev->current_cmd = cmd_type;  // Store current command for read operations

    // Start transmission
    HAL_StatusTypeDef hal_status = GPS_TRANSMIT(dev, dev->configs.device_address, dev->tx_buffer, dev->tx_size);
    
    if (hal_status != HAL_OK) {
        return UBLOX_I2C_ERROR;
    }
    
    return UBLOX_OK;
}


/**
 * @brief Generic non-blocking read interface for MAX-M10S GPS
 * @param dev Pointer to MAX-M10S device structure
 * @return UBLOX_OK if read initiated successfully; else error code
 */
gps_status_e max_m10s_read(max_m10s_dev_s *dev) {
    // Validate parameters
    if (dev == NULL || !dev->initialized) {
        return UBLOX_INVALID_PARAM;
    }
    // Determine expected response size based on the current command
    switch (dev->current_cmd) {
        case GPS_CMD_PVT:
            dev->rx_size = UBX_HEADER_LENGTH + UBX_NAV_PVT_LEN + UBX_CHECKSUM_LENGTH;
            break;
        default:
            return UBLOX_INVALID_PARAM;
    }
    // Start reception
    HAL_StatusTypeDef hal_status = GPS_RECEIVE(dev, dev->configs.device_address, dev->rx_buffer, dev->rx_size);

    if (hal_status != HAL_OK) {
        return UBLOX_I2C_ERROR;
    }
    return UBLOX_OK;
}

/**
 * @brief Validate a received packet from the GPS device
 * @param dev Pointer to MAX-M10S device structure
 * @param cmd_type Type of command for which the response should be validated
 * @return UBLOX_OK if packet is valid; else error code
 */
gps_status_e max_m10s_validate_response(max_m10s_dev_s *dev, gps_cmd_type_e cmd_type) {
    // Validate parameters
    if (dev == NULL || !dev->initialized || dev->rx_size == 0) {
        return UBLOX_INVALID_PARAM;
    }

    // Check for minimum packet size
    if (dev->rx_size < UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH) {
        return UBLOX_ERROR;
    }

    // Validate based on command type
    switch (cmd_type) {
        case GPS_CMD_PVT:
            return ubx_validate_packet(dev->rx_buffer, dev->rx_size, UBX_CLASS_NAV, UBX_NAV_PVT);
        
        // Add cases for other command types as needed
        
        default:
            return UBLOX_INVALID_PARAM;
    }
}



gps_status_e max_m10s_config_meas_rate(max_m10s_dev_s *dev, uint16_t rate) {
    if (dev == NULL || !dev->initialized) {
        return UBLOX_INVALID_PARAM;
    }
    if(rate < 25 || rate > 1000) {
        return UBLOX_INVALID_PARAM; // Rate must be between 25ms (40Hz) and 1000ms (1Hz)
    }
    uint16_t msg_size = ubx_prepare_config_cmd_u16(dev->tx_buffer, UBX_CFG_RATE_MEAS, rate);
    dev->tx_size = msg_size;

    HAL_StatusTypeDef hal_status = GPS_TRANSMIT(dev, dev->configs.device_address, dev->tx_buffer, dev->tx_size);
    if (hal_status != HAL_OK) {
        return UBLOX_I2C_ERROR;
    }

    I2C_WAIT(dev); 

    dev->configs.delay_blocking(1000); // 1 second delay

    hal_status = GPS_RECEIVE(dev, dev->configs.device_address, dev->rx_buffer, 10); // Expecting 10 bytes for ACK response
    if (hal_status != HAL_OK) {
        return UBLOX_I2C_ERROR; // Error status code
    }
    I2C_WAIT(dev);

    return ubx_validate_ack(dev->rx_buffer, 10, UBX_CLASS_CFG, UBX_CFG_VALSET);
}

//************** PRIVATE FUNCTIONS **************************************/
gps_status_e i2c_wait_for_complete(max_m10s_dev_s *dev) {
    // This function will wait for the I2C operation to complete
    uint32_t timeout = 1000; // Timeout in ms
    uint32_t start_time = HAL_GetTick();
    while(HAL_I2C_GetState(dev->configs.hi2c) != HAL_I2C_STATE_READY) {
        if (HAL_GetTick() - start_time > timeout) {
            return UBLOX_TIMEOUT; // Timeout error
        }
    }
    return UBLOX_OK;
}

