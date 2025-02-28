#include "ubx_protocol.h"
#include "target_config.h"
#include "stdlib.h"


/*******************************************************************************
 * Private Function Prototypes
 ******************************************************************************/
/**
 * @brief Parses ACK/NACK response from the u-blox module. An ACK/NACK response is
 *      expected after sending a Configurate Message. See in interface description pg.
 *      124
 * @param proto Pointer to UBX protocol structure
 * @return gps_status_e UBLOX_OK if ACK received, UBLOX_ERROR if NACK or invalid response
 */
static gps_status_e parse_ack_response(ubx_protocol_t* proto);


/**
 * @brief Retrieves navigation PVT data from the GNSS module which has a large array of
 *        useful data; this function alone should provide any type of data needed.
 * @param proto Pointer to UBX protocol structure
 * @return Returns UBLOX_OK if request successful; else UBLOX_ERROR_XX
 */
static gps_status_e ubx_get_pvt(ubx_protocol_t* proto);

/*******************************************************************************
 * Public Function Implementations
 ******************************************************************************/

/**
  * @brief Sends a UBX command packet to the GPS module
  * @details Calculates checksum, formats packet according to UBX protocol,
  *          and handles I2C transmission
  */
gps_status_e send_ubx_command(ubx_protocol_t* proto, bool ack_only){
    #ifdef DEBUG
        debug_print("Sending UBX command...\r\n");
    #endif
    // Send command to GPS module
    uint16_t size = UBX_HEADER_LENGTH + proto->frame.len + UBX_CHECKSUM_LENGTH;
    uint8_t buff[size];

    buff[0] = UBX_SYNC_CHAR_1;
    buff[1] = UBX_SYNC_CHAR_2;
    buff[2] = proto->frame.cls;
    buff[3] = proto->frame.id;
    buff[4] = proto->frame.len & 0xFF;     //LSB
    buff[5] = proto->frame.len >> 8;       // MSB

    memcpy(&buff[6], proto->frame.payload.raw, proto->frame.len);

    buff[size-2] = proto->frame.checksumA;
    buff[size-1] = proto->frame.checksumB;

    // Send the data on bus
    if(HAL_I2C_Master_Transmit(&hi2c1, UBLOX_I2C_ADDR, buff, size, I2C_TIMEOUT) != HAL_OK){
        return UBLOX_ERROR;
    }

    // After sending command, mark frame as processed since it's no longer needed
    proto->frame_state = UBX_FRAME_PROCESSED;

    #ifdef DEBUG
        debug_print("Sent UBX command...\r\n");
    #endif


    // Update state variables
    proto->state.last_cls = proto->frame.cls;
    proto->state.last_id = proto->frame.id;

    HAL_Delay(1000);   // IMPORTANT: A small delay is needed to allow the module to process the command
                          // before the requestor can read the response. I tried 100 ms and adjusted until
                          // finding that 1000 ms was the sweet spot and allowed the module enough time to process
                          // a command.

    // If ACK is expected, verify response
    return ack_only ? parse_ack_response(proto) : UBLOX_OK;
}

/**
  * @brief Parses ACK/NACK response from the u-blox module
  * @return gps_status_e UBX_OK if ACK received, UBX_ERROR if NACK or invalid response
  */
static gps_status_e parse_ack_response(ubx_protocol_t* proto) {
    uint8_t buff[UBX_HEADER_LENGTH + UBX_ACK_ACK_LEN + UBX_CHECKSUM_LENGTH];

    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, sizeof(buff), I2C_TIMEOUT) != HAL_OK) {
        return UBLOX_ERROR;
    }

    return packet_validate_ack(buff, sizeof(buff), proto->frame.cls, proto->frame.id);
}


/**
 * @brief Retrieves navigation PVT data from the GNSS module
 * @details Requests NAV-PVT message which provides position, velocity, and time data
 * @param packet Pointer to UBX packet structure to store response
 * @return Returns UBLOX_OK if request successful; else UBLOX_ERROR_XX
 */
gps_status_e ubx_get_pvt(ubx_protocol_t* proto){

    gps_status_e status = packet_prepare_command(&proto->frame,
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

    status = send_ubx_command(proto, false);
    if (status != UBLOX_OK) {
        #ifdef DEBUG
            debug_print("Error sending NAV-PVT request\r\n");
        #endif
        return status;
    }

    // Read response
    uint8_t buff[UBX_HEADER_LENGTH + UBX_NAV_PVT_LEN + UBX_CHECKSUM_LENGTH];
    if (HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, sizeof(buff), I2C_TIMEOUT) != HAL_OK) {
        #ifdef DEBUG
            debug_print("I2C HAL Error receiving NAV-PVT response\r\n");
        #endif
        return UBLOX_I2C_ERROR;
    }
    // Validate response
    status = packet_validate_response(buff, sizeof(buff), UBX_CLASS_NAV, UBX_NAV_PVT);
    if (status != UBLOX_OK) {
        return status;
    }

    status = packet_copy_payload(&proto->frame, buff, UBX_NAV_PVT_LEN);

    if (status != UBLOX_OK) {
        return status;
    }
    proto->frame_state = UBX_FRAME_IN_USE;

    return UBLOX_OK;
}

/**
 * @brief Extracts data from the UBX packet and stores it in the GPS data structure
 * @param packet Pointer to UBX packet structure
 * @param type Type of data to extract
 * @param data Pointer to GPS data structure to store extracted data
 * @return Returns UBLOX_OK if data extracted successfully; else UBLOX_ERROR_XX
 */
gps_status_e ubx_extract_pvt_data(ubx_protocol_t* proto, gps_data_type_e type, gps_data_t* data) {
    if (!proto || !data) {
        return UBLOX_ERROR;
    }
    // Check if new data is needed
    bool need_new_data = false;

    if(proto->frame_state == UBX_FRAME_EMPTY) {
        need_new_data = true;
    } else if(proto->frame_state == UBX_FRAME_PROCESSED) {
        need_new_data = true;
    } else if(proto->frame_state == UBX_FRAME_RECEIVED || proto->frame_state == UBX_FRAME_IN_USE) {
        if(proto->frame.cls == UBX_CLASS_NAV || proto->frame.id == UBX_NAV_PVT) {
            need_new_data = false; // Already have the data
        }
        else{
            // Different message is currently in use
            return UBLOX_FRAME_IN_USE;
        }
    }

    if(need_new_data) {
        gps_status_e status = ubx_get_pvt(proto);
        if(status != UBLOX_OK) {
            return status;
        }
    }

    switch (type) {
        case GPS_DATA_POSITION:
            data->position.latitude = proto->frame.payload.nav_pvt.lat;      // In degrees * 10^7
            data->position.longitude = proto->frame.payload.nav_pvt.lon;     // In degrees * 10^7
            data->position.altitude = proto->frame.payload.nav_pvt.hMSL;     // In  mm above MSL;
            data->position.satellites = proto->frame.payload.nav_pvt.numSV;
            data->position.fix_type = proto->frame.payload.nav_pvt.fixType;
            data->position.valid = (proto->frame.payload.nav_pvt.flags.bits.gnssFixOK == 1) &&
                                 (proto->frame.payload.nav_pvt.valid.bits.validDate == 1) &&
                                 (proto->frame.payload.nav_pvt.valid.bits.validTime == 1);
            break;

        case GPS_DATA_TIME:
            data->time.year = proto->frame.payload.nav_pvt.year;
            data->time.month = proto->frame.payload.nav_pvt.month;
            data->time.day = proto->frame.payload.nav_pvt.day;
            data->time.hour = proto->frame.payload.nav_pvt.hour;
            data->time.minute = proto->frame.payload.nav_pvt.min;
            data->time.second = proto->frame.payload.nav_pvt.sec;
            data->time.valid = proto->frame.payload.nav_pvt.valid.bits.validTime &&
                             proto->frame.payload.nav_pvt.valid.bits.validDate;
            break;

        default:
            return UBLOX_ERROR;
    }

    return UBLOX_OK;
}

gps_status_e ubx_free_frame(ubx_protocol_t* proto) {
    if(!proto) {
        return UBLOX_ERROR;
    }
    memset(&proto->frame, 0, sizeof(proto->frame));
    proto->frame_state = UBX_FRAME_EMPTY;
    return UBLOX_OK;
}

/**
 * @brief Retrieves navigation status from the GNSS module
 * @details Requests NAV-STATUS message which provides receiver navigation status
 *          including fix type, validity flags, and time information
 * @param packet Pointer to UBX packet structure to store response
 * @return Returns UBLOX_OK if request successful; else UBLOX_ERROR_XX
 */
gps_status_e ubx_get_nav_status(ubx_protocol_t* proto) {
    if(!proto) {
        return UBLOX_ERROR;
    }

    gps_status_e status = packet_prepare_command(&proto->frame, UBX_CLASS_NAV, UBX_NAV_STATUS, NULL, 0);
    if(status != UBLOX_OK) {
        return status;
    }
    #ifdef DEBUG
        debug_print("Requesting NAV-STATUS...\r\n");
    #endif

    status = send_ubx_command(proto, false);
    if(status != UBLOX_OK) {
        #ifdef DEBUG
            debug_print("Error sending NAV-STATUS request\r\n");
        #endif
        return status;
    }
    // Read response
    uint8_t buff[UBX_HEADER_LENGTH + UBX_NAV_STATUS_LEN + UBX_CHECKSUM_LENGTH];

    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, sizeof(buff), I2C_TIMEOUT) != HAL_OK) {
        #ifdef DEBUG
            debug_print("I2C HAL Error receiving NAV-STATUS response\r\n");
        #endif
        return UBLOX_I2C_ERROR;
    }
    status = packet_validate_response(buff, sizeof(buff), UBX_CLASS_NAV, UBX_NAV_STATUS);
    if(status != UBLOX_OK) {
        return status;
    }
    status = packet_copy_payload(&proto->frame, buff, UBX_NAV_STATUS_LEN);
    if(status != UBLOX_OK) {
        return status;
    }

    if(proto->frame.payload.nav_status.flags.bits.gpsFixOk != 1) {
        #ifdef DEBUG
            debug_print("GPS fix not valid\r\n");
        #endif
        return UBLOX_NO_FIX;
    }
    if((proto->frame.payload.nav_status.flags.bits.wknSet && proto->frame.payload.nav_status.flags.bits.towSet) != 1) {
        #ifdef DEBUG
            debug_print("Time not valid\r\n");
        #endif
        return UBLOX_INVALID_TIME;
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
  * @return gps_status_e UBLOX_OK if configuration successful, error code otherwise
  *
  * @note Configuration requires acknowledgment from module to confirm success
  */
gps_status_e ubx_configure_i2c(ubx_protocol_t* proto) {

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
    gps_status_e status = packet_prepare_command(&proto->frame, UBX_CLASS_CFG, UBX_CFG_VALSET, payload_en_ubx, sizeof(payload_en_ubx));

    if(status != UBLOX_OK){
        return status;
    }
    // Send command and expect ACK
    status = send_ubx_command(proto, true);

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
    status = packet_prepare_command(&proto->frame, UBX_CLASS_CFG, UBX_CFG_VALSET, payload_disable_nmea, sizeof(payload_disable_nmea));

    if(status != UBLOX_OK){
        return status;
    }

    return send_ubx_command(proto, true); // Expect ACK
}

/**
 * @brief
 *
 * @param proto Pointer to UBX protocol structure
 * @return gps_status_e UBLOX_OK if reset successful, error code otherwise
 */
gps_status_e ubx_reset(ubx_protocol_t* proto) {
    if (!proto) {
        return UBLOX_ERROR;
    }

    // Prepare the reset payload
    uint8_t reset_payload[] = {
        0xFF,   // Clear all BBR sections
        0xFF,   // Clear all BBR sections
        0x00,   // Hardware reset
        0x00    // Reserved byte
    };

    // Prepare command packet
    gps_status_e status = packet_prepare_command(&proto->frame,
                                               UBX_CLASS_CFG,
                                               UBX_CFG_RST,
                                               reset_payload,
                                               sizeof(reset_payload));
    if (status != UBLOX_OK) {
        return status;
    }

    // Send command and don't expect ACK since device will reset
    status = send_ubx_command(proto, false);

    // Reset protocol state
    status = ubx_free_frame(proto);

    return status;
}
