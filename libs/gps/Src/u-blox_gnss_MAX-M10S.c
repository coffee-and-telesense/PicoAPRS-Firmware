/*******************************************************************************
 * @file: u-blox_gnss_MAX-M10S.c
 * @brief: Implementation of the u-blox MAX-M10S GNSS module driver using UBX protocol
 *         Provides initialization and communication functions for configuring and
 *         retrieving data from the GPS module over I2C.
 *
 * @note: This implementation focuses on UBX protocol support and I2C communication.
 *        NMEA protocol support may be added in future versions.
 *
 * @author: Reece Wayt
 * @date: January 13, 2025
 * @version: 1.0
 *
 * @dependencies:
 *   - STM32 HAL I2C driver
 *   - STM32 HAL UART driver (for debug output)
 ******************************************************************************/

#include "u-blox_gnss_MAX-M10S.h"


/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
static ublox_status_e ublox_send_command(bool expect_ack_only);
static ublox_status_e set_ubx_i2c_output(ublox_comm_type_e comm_settings);
static ublox_status_e parse_ack_response(void);
static ublox_status_e process_nav_status(void);
static ublox_status_e validate_nav_pvt(void);

/*******************************************************************************
 * Private & Extern Variables
 ******************************************************************************/
static bool ublox_initialized = false;
static ubx_packet_t ubx_packet;

/*******************************************************************************
 * Public Function Implementations
 ******************************************************************************/

/**
 * @brief Initializes the u-blox GNSS module
 * @details Initializes I2C communication, clears packet structure, and configures
 *          module for UBX protocol output.
 * @return ublox_status_e Initialization status
 */

ublox_status_e ublox_init(void){
    // Initialize the I2C peripheral
    ublox_comm_type_e comm_settings = UBX;
    MX_I2C1_Init();
    // Zero packet structure
    memset(&ubx_packet, 0, sizeof(ubx_packet_t));
    ublox_status_e status = set_ubx_i2c_output(comm_settings);
    if(status != UBLOX_OK) {
        return status;
    }
    ublox_initialized = true;
    return UBLOX_OK;
}

/**
 * @brief Retrieves navigation status from the GNSS module
 * @details Requests NAV-STATUS message which provides receiver navigation status
 *          including fix type, validity flags, and time information
 * @return Returns UBLOX_OK if fix is valid and time is valid; else UBLOX_ERROR_XX
 */
ublox_status_e ublox_get_nav_status(void){
    bool ack_only = false; // Expect full response

    if(!ublox_initialized){
        #ifdef DEBUG
            debug_print("GPS module not initialized!\r\n");
        #endif
        return UBLOX_NOT_INITED;
    }

    if(ubx_packet.valid){
        #ifdef DEBUG
            debug_print("Packet is valid, previous frame not processed correctly\r\n");
        #endif
        return UBLOX_PACKET_NEEDS_PROCESSING;
    }
    ublox_status_e status = packet_prepare_command(&ubx_packet, UBX_CLASS_NAV, UBX_NAV_STATUS, NULL, 0);

    if(status != UBLOX_OK) {
        return status;
    }

    #ifdef DEBUG
        debug_print("Requesting NAV-STATUS...\r\n");
    #endif

    if(ublox_send_command(ack_only) != UBLOX_OK){
        #ifdef DEBUG
            debug_print("Error sending NAV-STATUS request\r\n");
        #endif
        return UBLOX_ERROR;
    }

    // Command sent now read the response
    uint8_t buff[UBX_HEADER_LENGTH + UBX_NAV_STATUS_LEN + UBX_CHECKSUM_LENGTH];

    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, sizeof(buff), I2C_TIMEOUT) != HAL_OK){
        #ifdef DEBUG
            debug_print("I2C HAL Error receiving NAV-STATUS response\r\n");
        #endif
        return UBLOX_ERROR;
    }

    status = packet_validate_response(buff, sizeof(buff), UBX_CLASS_NAV, UBX_NAV_STATUS);
    if(status != UBLOX_OK) {
        return status;
    }
    status = packet_copy_payload(&ubx_packet, buff, UBX_NAV_STATUS_LEN);
    if(status != UBLOX_OK) {
        return status;
    }

    return process_nav_status();
}

/**
 * @brief Retrieves navigation PVT data from the GNSS module
 * @details Requests NAV-PVT message which provides position, velocity, and time data
 * @return Returns UBLOX_OK if fix is valid and time is valid; else UBLOX_ERROR_XX
 */
ublox_status_e ublox_get_pvt(void){
    bool ack_only = false;

    if(!ublox_initialized) {
        #ifdef DEBUG
            debug_print("GPS module not initialized!\r\n");
        #endif
        return UBLOX_NOT_INITED;
    }

    if(ubx_packet.valid) {
        #ifdef DEBUG
            debug_print("Packet is valid, previous frame not processed correctly\r\n");
        #endif
        return UBLOX_PACKET_NEEDS_PROCESSING;
    }

    // Prepare command packet
    ublox_status_e status = packet_prepare_command(&ubx_packet,
                                                 UBX_CLASS_NAV,
                                                 UBX_NAV_PVT,
                                                 NULL,
                                                 0);
    if(status != UBLOX_OK) {
        return status;
    }

    #ifdef DEBUG
        debug_print("Requesting NAV-PVT...\r\n");
    #endif

    if(ublox_send_command(ack_only) != UBLOX_OK) {
        #ifdef DEBUG
            debug_print("Error sending NAV-PVT request\r\n");
        #endif
        return UBLOX_ERROR;
    }

    // Read response
    uint8_t buff[UBX_HEADER_LENGTH + UBX_NAV_PVT_LEN + UBX_CHECKSUM_LENGTH];

    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, sizeof(buff), I2C_TIMEOUT) != HAL_OK) {
        #ifdef DEBUG
            debug_print("I2C HAL Error receiving NAV-PVT response\r\n");
        #endif
        return UBLOX_I2C_ERROR;
    }

    // Validate response
    status = packet_validate_response(buff, sizeof(buff), UBX_CLASS_NAV, UBX_NAV_PVT);
    if(status != UBLOX_OK) {
        return status;
    }

    // Copy payload
    status = packet_copy_payload(&ubx_packet, buff, UBX_NAV_PVT_LEN);
    if(status != UBLOX_OK) {
        return status;
    }

    return validate_nav_pvt();
}

/**
 * @brief Get lat, lon, height, and time from the gnss module
 * @details The ublox module returns the lat and lon in deg * 1e-7, and for height this returns
 *          the height above mean sea level in mm. All values from the module are 32 bit, signed integers (twos-complement).
 *
 */
ublox_status_e ublox_get_curr_position(int32_t *lat, int32_t *lon, int32_t *height) {
    if(!ubx_packet.valid){
        #ifdef DEBUG
            debug_print("Packet is invalid, cannot get position\r\n");
        #endif
        return UBLOX_INVALID_DATA;
    }
    *lat = ubx_packet.payload.nav_pvt.lat;
    *lon = ubx_packet.payload.nav_pvt.lon;
    *height = ubx_packet.payload.nav_pvt.hMSL;

    // Reset the packet to invalid since processing is complete
    ubx_packet.valid = false;

    return UBLOX_OK;
}

/*******************************************************************************
 * Private Function Implementations
 ******************************************************************************/

/**
 * @brief Sends a UBX command packet to the GPS module
 * @details Calculates checksum, formats packet according to UBX protocol,
 *          and handles I2C transmission
 */
static ublox_status_e ublox_send_command(bool expect_ack_only){
    // Holds the last message class and ID state
    static uint8_t last_msg_class = 0;
    static uint8_t last_msg_id = 0;
    static uint32_t delay_ms = 1000; // Delay after sending command, adjusts based on previous command

    #ifdef DEBUG
        debug_print("Sending UBX command...\r\n");
    #endif

    uint16_t size = UBX_HEADER_LENGTH + ubx_packet.len + UBX_CHECKSUM_LENGTH;
    uint8_t buff[size];

    buff[0] = UBX_SYNC_CHAR_1;
    buff[1] = UBX_SYNC_CHAR_2;
    buff[2] = ubx_packet.cls;
    buff[3] = ubx_packet.id;
    buff[4] = ubx_packet.len & 0xFF;     //LSB
    buff[5] = ubx_packet.len >> 8;       // MSB

    memcpy(&buff[6], ubx_packet.payload.raw, ubx_packet.len);

    // Calculate checksum
    calc_check_sum(&buff[2], ubx_packet.len + 4, &buff[size-2], &buff[size-1]);

    // Send the data on bus
    if(HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDR, buff, size, I2C_TIMEOUT) != HAL_OK){
        return UBLOX_ERROR;
    }
    // After sending command always clear the valid bits since we expect a
    // response from the module
    ubx_packet.valid = 0;

    #ifdef DEBUG
        debug_print("Sent UBX command...\r\n");
    #endif

    if (ubx_packet.cls == last_msg_class && ubx_packet.id == last_msg_id) {
        // If the same message is sent twice in a row, decrease delay
        if(ubx_packet.cls == UBX_CLASS_CFG) // CFG messages are slow, keep this here.
            delay_ms = 1000; // try this to start
        else {
            delay_ms = 500; // try this to start
        }
    } else {
        // Reset delay if a different message is sent
        delay_ms = 1000;
    }

    // Update state variables
    last_msg_class = ubx_packet.cls;
    last_msg_id = ubx_packet.id;

    HAL_Delay(delay_ms); // IMPORTANT: A small delay is needed to allow the module to process the command
                         // before the requestor can read the response. I tried 100 ms and adjusted until
                         // finding that 1000 ms was the sweet spot and allowed the module enough time to process
                         // a command. Sequential commands to the same class and ID can be read a bit faster.

    if(expect_ack_only) {
        // Parse the ACK/NACK response
        return parse_ack_response();
    }
    return UBLOX_OK;
}
/**
 * @brief Process NAV Status response from the u-blox module
 * @return ublox_status_e UBX_OK if gps fix is valid, and time is valid; else UBX_ERROR
 */
static ublox_status_e process_nav_status(void) {
    // Simple error check to see that the packet is valid
    if(!ubx_packet.valid){
        return UBLOX_INVALID_DATA;
    }
    // Check if fix is valid
    if(ubx_packet.payload.nav_status.flags.bits.gpsFixOk != 1) {
        #ifdef DEBUG
            debug_print("GPS fix not valid\r\n");
        #endif
        return UBLOX_ERROR;
    }
    // Check if time is valid
    if((ubx_packet.payload.nav_status.flags.bits.wknSet && ubx_packet.payload.nav_status.flags.bits.towSet) != 1) {
        #ifdef DEBUG
            debug_print("Time not valid\r\n");
        #endif
        return UBLOX_ERROR;
    }
    // Reset the packet to invalid since processing is complete
    ubx_packet.valid = 0;
    return UBLOX_OK;
}

/**
 * @brief validate_nav_pvt helper function to validate the packet before exposing the data to the user
 *
 */
static ublox_status_e validate_nav_pvt(void) {
    // Simple error check to see that the packet is valid
    if(!ubx_packet.valid){
        return UBLOX_INVALID_DATA;
    }
    // Check if date and time are valid
    if(ubx_packet.payload.nav_pvt.valid.bits.validDate != 1 || ubx_packet.payload.nav_pvt.valid.bits.validTime != 1) {
        #ifdef DEBUG
            debug_print("Date or time not valid\r\n");
        #endif
        // Set to invalid
        ubx_packet.valid = 0;
        return UBLOX_ERROR;
    }
    // Check if fix is valid
    if(ubx_packet.payload.nav_pvt.flags.bits.gnssFixOK != 1) {
        #ifdef DEBUG
            debug_print("GPS fix not valid\r\n");
        #endif
        // Set to invalid
        ubx_packet.valid = 0;
        return UBLOX_ERROR;
    }
    // Checks passed, return OK. User can now access the data
    return UBLOX_OK;
}

/**
 * @brief Parses ACK/NACK response from the u-blox module
 * @param rx_buff Pointer to received data buffer
 * @param rx_len Length of received data
 * @return ublox_status_e UBX_OK if ACK received, UBX_ERROR if NACK or invalid response
 */
static ublox_status_e parse_ack_response(void) {
    uint8_t buff[UBX_HEADER_LENGTH + UBX_ACK_ACK_LEN + UBX_CHECKSUM_LENGTH];
    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, sizeof(buff), I2C_TIMEOUT) != HAL_OK){
        return UBLOX_ERROR;
    }

    ublox_status_e status = packet_validate_response(buff, sizeof(buff), UBX_CLASS_ACK, UBX_ACK_ACK);

    if (status != UBLOX_OK) {
        if (buff[3] == UBX_ACK_NACK) {
            #ifdef DEBUG
                debug_print("Received NACK response\r\n");
            #endif
            return UBLOX_ERROR;
        }
        return status;
    }

    // Verify ACK payload matches our sent command
    if(buff[6] != ubx_packet.cls || buff[7] != ubx_packet.id) {
        #ifdef DEBUG
            debug_print("ACK payload does not match sent command\r\n");
        #endif
        return UBLOX_ERROR;
    }

    return UBLOX_OK;

}

/**
 * @brief Configures the I2C output protocol settings of the u-blox module
 *
 * @details Sets up the module to use specified protocol (UBX/NMEA) over I2C.
 *          Uses CFG-VALSET message to configure the I2COUTPROT setting.
 *          Configuration is stored in battery-backed RAM (BBRAM) to persist
 *          during power cycles while backup power is maintained.
 *
 * @param comm_settings Communication protocol selection (UBX or NMEA)
 * @return ublox_status_e UBLOX_OK if configuration successful, error code otherwise
 *
 * @note Configuration requires acknowledgment from module to confirm success
 */
static ublox_status_e set_ubx_i2c_output(ublox_comm_type_e comm_settings) {
    //Clear packet structure
    if(comm_settings != UBX){
        return UBLOX_ERROR;
    }

    // Setup payload for enabling UBX protocol
    uint8_t payload_en_ubx[] = {
        0x00,   // Version
        0x11,   // Layer = BBRAM (0x10) | RAM (0x01)
        0x00,   // Reserved
        0x00,   // Reserved
        0x01,   // LSB: Key ID (0x10720001)
        0x00,   // CFG-I2COUTPROT-UBX
        0x72,
        0x10,   // MSB: Key ID
        0x01    // Value = UBX protocol
    };
    ublox_status_e status = packet_prepare_command(&ubx_packet, UBX_CLASS_CFG, UBX_CFG_VALSET, payload_en_ubx, sizeof(payload_en_ubx));

    if(status != UBLOX_OK){
        return status;
    }
    // Send command and expect ACK
    status = ublox_send_command(true);

    if(status != UBLOX_OK){
        return status;
    }

    // Setup payload for disabling NMEA protocol
    uint8_t payload_disable_nmea[] = {
        0x00,   // Version
        0x11,   // Layer = BBRAM (0x10) | RAM (0x01)
        0x00,   // Reserved
        0x00,   // Reserved
        0x02,   // LSB: Key ID (0x10720002)
        0x00,   // CFG-I2COUTPROT-NMEA
        0x72,
        0x10,   // MSB: Key ID
        0x00    // Value = NMEA protocol (disabled)
    };

    // Prepare command packet
    status = packet_prepare_command(&ubx_packet, UBX_CLASS_CFG, UBX_CFG_VALSET, payload_disable_nmea, sizeof(payload_disable_nmea));

    if(status != UBLOX_OK){
        return status;
    }

    return ublox_send_command(true); // Expect ACK
}


ublox_status_e ublox_reset(void) {
    // Prepare the reset payload
    uint8_t reset_payload[] = {
        0xFF,   // Clear all BBR sections
        0xFF,   // Clear all BBR sections
        0x00,   // Hardware reset
        0x00    // Reserved byte
    };

    // Use packet handler to prepare the command
    ublox_status_e status = packet_prepare_command(&ubx_packet,
                                                 UBX_CLASS_CFG,
                                                 UBX_CFG_RST,  // Reset command ID
                                                 reset_payload,
                                                 sizeof(reset_payload));
    if(status != UBLOX_OK) {
        return status;
    }

    return ublox_send_command(false);  // Expect ACK
}
