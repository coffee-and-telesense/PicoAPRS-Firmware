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
static void calc_check_sum(void);
static uint16_t set_transaction_size(uint16_t len);
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
        return UBLOX_NOT_INITIALIZED;
    }

    if(ubx_packet.valid){
        #ifdef DEBUG
            debug_print("Packet is valid, previous frame not processed correctly\r\n");
        #endif
        return UBLOX_PACKET_NEEDS_PROCESSING;
    }
    // Always clear the packet before sending a new request
    memset(&ubx_packet, 0, UBX_PACKET_LENGTH);

    // Setup packet for nav status request
    ubx_packet.cls = UBX_CLASS_NAV;
    ubx_packet.id = UBX_NAV_STATUS;

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
    uint16_t size = UBX_HEADER_LENGTH + UBX_NAV_STATUS_LEN + UBX_CHECKSUM_LENGTH;
    uint8_t buff[size];

    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, size, I2C_TIMEOUT) != HAL_OK){
        #ifdef DEBUG
            debug_print("I2C HAL Error receiving NAV-STATUS response\r\n");
        #endif
        return UBLOX_ERROR;
    }

    // Verify sync chars
    if (buff[0] != UBX_SYNC_CHAR_1 || buff[1] != UBX_SYNC_CHAR_2) {
        return UBLOX_INVALID_DATA;
    }
    // Verify message class and ID
    if (buff[2] != UBX_CLASS_NAV || buff[3] != UBX_NAV_STATUS) {
        return UBLOX_INVALID_DATA;
    }
    // Copy payload into packet structure
    memcpy(&ubx_packet.payload.nav_status, &buff[UBX_HEADER_LENGTH], UBX_NAV_STATUS_LEN);

    // Set frame to valid
    ubx_packet.valid = true;

    // Returns UBX_OK if fix is valid and time is valid
    return process_nav_status();
}

/**
 * @brief Retrieves navigation PVT data from the GNSS module
 * @details Requests NAV-PVT message which provides position, velocity, and time data
 * @return Returns UBLOX_OK if fix is valid and time is valid; else UBLOX_ERROR_XX
 */
ublox_status_e ublox_get_pvt(void){
    bool ack_only = false; // Expect full response

    if(!ublox_initialized){
        #ifdef DEBUG
            debug_print("GPS module not initialized!\r\n");
        #endif
        return UBLOX_NOT_INITIALIZED;
    }
    if(ubx_packet.valid){
        #ifdef DEBUG
            debug_print("Packet is valid, previous frame not processed correctly\r\n");
        #endif
        return UBLOX_PACKET_NEEDS_PROCESSING;
    }
    // Always clear the packet before sending a new request
    memset(&ubx_packet, 0, UBX_PACKET_LENGTH);

    // Setup packet for nav PVT request
    ubx_packet.cls = UBX_CLASS_NAV;
    ubx_packet.id = UBX_NAV_PVT;

    #ifdef DEBUG
        debug_print("Requesting NAV-PVT...\r\n");
    #endif

    if(ublox_send_command(ack_only) != UBLOX_OK){
        #ifdef DEBUG
            debug_print("Error sending NAV-PVT request\r\n");
        #endif
        return UBLOX_ERROR;
    }

    // Command sent now read the response
    uint16_t size = UBX_HEADER_LENGTH + UBX_NAV_PVT_LEN + UBX_CHECKSUM_LENGTH;
    uint8_t buff[size];

    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, size, I2C_TIMEOUT) != HAL_OK){
        #ifdef DEBUG
            debug_print("I2C HAL Error receiving NAV-PVT response\r\n");
        #endif
        return UBLOX_ERROR;
    }

    // Verify sync chars
    if (buff[0] != UBX_SYNC_CHAR_1 || buff[1] != UBX_SYNC_CHAR_2) {
        return UBLOX_INVALID_DATA;
    }
    // Verify message class and ID
    if (buff[2] != UBX_CLASS_NAV || buff[3] != UBX_NAV_PVT) {
        return UBLOX_INVALID_DATA;
    }
    // Copy payload into packet structure
    memcpy(&ubx_packet.payload.nav_pvt, &buff[UBX_HEADER_LENGTH], UBX_NAV_PVT_LEN);

    // Set frame to valid
    ubx_packet.valid = true;

    // Returns UBX_OK if fix is valid and time is valid
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
    calc_check_sum();

    #ifdef DEBUG
        debug_print("Sending UBX command...\r\n");
    #endif

    //Calculate the size of the transaction (header + payload + checksum)
    uint16_t size = set_transaction_size(ubx_packet.len);

    // Should never be zero
    if (size == 0){
        return UBLOX_ERROR;
    }

    uint8_t buff[size];
    buff[0] = UBX_SYNC_CHAR_1;
    buff[1] = UBX_SYNC_CHAR_2;
    buff[2] = ubx_packet.cls;
    buff[3] = ubx_packet.id;
    buff[4] = ubx_packet.len & 0xFF;     //LSB
    buff[5] = ubx_packet.len >> 8;       // MSB
    uint16_t i = 0;
    for(; i < ubx_packet.len; i++){
        // TODO: Add error check here so payload and i don't go out of bounds
        buff[i + 6] = ubx_packet.payload.raw[i];
    }
    buff[i + 6] = ubx_packet.checksumA;
    buff[i + 7] = ubx_packet.checksumB;

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
    HAL_Delay(1000); // IMPORTANT: A small delay is needed to allow the module to process the command
                     // before the requestor can read the response. I tried 100 ms and adjusted until
                     // finding that 1000 ms was the sweet spot and allowed the module enough time to process
                     // a command.

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
    uint16_t size = 10;                        // Ack response is always 8 bytes
    uint8_t buff[size];
    if(HAL_I2C_Master_Receive(&hi2c1, UBLOX_I2C_ADDR, buff, size, I2C_TIMEOUT) != HAL_OK){
        return UBLOX_ERROR;
    }

    // Check sync chars
    if (buff[0] != UBX_SYNC_CHAR_1 || buff[1] != UBX_SYNC_CHAR_2) {
        #ifdef DEBUG
            debug_print("Invalid sync chars in ACK response\r\n");
        #endif
        return UBLOX_INVALID_DATA;
    }
    // Check class and ID for ACK-ACK (0x05 0x01) or ACK-NACK (0x05 0x00)
    if (buff[2] != UBX_CLASS_ACK) {
        #ifdef DEBUG
            debug_print("Invalid class in ACK response\r\n");
        #endif
        return UBLOX_INVALID_DATA;
    }
    if (buff[3] == UBX_ACK_NACK){
        #ifdef DEBUG
            debug_print("Received NACK\r\n");
        #endif
        return UBLOX_NACK_ERROR;
    }
    else if (buff[3] == UBX_ACK_ACK) {
        #ifdef DEBUG
            debug_print("Received ACK\r\n");
        #endif
        // Length should be 2 bytes, skip these since we know if will always be 2 here
        if(buff[6] == ubx_packet.cls && buff[7] == ubx_packet.id){
            #ifdef DEBUG
                debug_print("ACK payload matches sent command\r\n");
            #endif
            return UBLOX_OK;
        }
        else{
            #ifdef DEBUG
                debug_print("ACK payload does not match sent command\r\n");
            #endif
            return UBLOX_ERROR;
        }
    }
    // Should never reach here, unknown response
    return UBLOX_ERROR;
}

/**
 * @brief Calculates Fletcher checksum for UBX packet
 *
 * @details Implements the Fletcher checksum algorithm for UBX protocol packets.
 *          Calculates checksums over class, ID, length, and payload bytes.
 *          The algorithm uses two running sums (checksumA and checksumB) where:
 *          - checksumA is a simple sum of bytes
 *          - checksumB is a sum of all checksumA values
 *
 * @param outgoing_ubx Pointer to UBX packet structure containing data to checksum
 *
 * @note Checksum calculation order:
 *       1. Class byte
 *       2. ID byte
 *       3. Length bytes (little endian)
 *       4. All payload bytes (if present)
 */
static void calc_check_sum() {
    // Reset checksums
    ubx_packet.checksumA = 0;
    ubx_packet.checksumB = 0;

    // Add class
    ubx_packet.checksumA += ubx_packet.cls;
    ubx_packet.checksumB += ubx_packet.checksumA;

    // Add ID
    ubx_packet.checksumA += ubx_packet.id;
    ubx_packet.checksumB += ubx_packet.checksumA;

    // Add length (2 bytes, little endian)
    ubx_packet.checksumA += (ubx_packet.len & 0xFF);
    ubx_packet.checksumB += ubx_packet.checksumA;

    ubx_packet.checksumA += (ubx_packet.len >> 8);
    ubx_packet.checksumB += ubx_packet.checksumA;

    // Add payload bytes if there are any
    if(ubx_packet.len > 0) {
        for(uint16_t i = 0; i < ubx_packet.len; i++) {
            ubx_packet.checksumA += ubx_packet.payload.raw[i];
            ubx_packet.checksumB += ubx_packet.checksumA;
        }
    }
}


/**
 * @brief Calculates and validates total transaction size for I2C communication
 *
 * @details Calculates the total packet size including:
 *          - 6 bytes header (2 sync + 1 class + 1 ID + 2 length)
 *          - N bytes payload
 *          - 2 bytes checksum
 *          Validates that the total size doesn't exceed maximum packet length
 *
 * @param len Length of payload in bytes
 * @return uint16_t Total transaction size if valid, 0 if size would exceed maximum
 *
 * @note Debug message is output if size validation fails
 */
static uint16_t set_transaction_size(uint16_t len){
    // Number of bytes written is payload length + 8 bytes for header and checksum
    uint16_t size = len + 8;
    if(size > UBX_PACKET_LENGTH){
        #ifdef DEBUG
            debug_print("Packet size exceeds maximum length!\r\n");
        #endif
        return 0; // Error
    }
    return size;
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
    memset(&ubx_packet, 0, sizeof(ubx_packet_t));

    // Setup packet for CFG-VALSET message
    ubx_packet.cls = UBX_CLASS_CFG;
    ubx_packet.id = UBX_CFG_VALSET;
    ubx_packet.len = 9;

    ubx_packet.payload.raw[0] = 0x00;   // Version
    ubx_packet.payload.raw[1] = 0x11;   // Layer = BBRAM (0x10) | RAM (0x01) see pg. 125 of Interface Description
    ubx_packet.payload.raw[2] = 0x00;   // Reserved
    ubx_packet.payload.raw[3] = 0x00;   // Reserved

    ubx_packet.payload.raw[4] = 0x01;   // LSB: Key ID (0x10720001)
    ubx_packet.payload.raw[5] = 0x00;   // CFG-I2COUTPROT-UBX
    ubx_packet.payload.raw[6] = 0x72;   //
    ubx_packet.payload.raw[7] = 0x10;   // MSB: Key ID
    ubx_packet.payload.raw[8] = 0x01;   // Value = UBX protocol

    ublox_status_e status = ublox_send_command(true);
    if(status != UBLOX_OK){
        return status;
    }

    // Set the I2C output protocol to UBX
    memset(&ubx_packet, 0, sizeof(ubx_packet_t));
    ubx_packet.cls = UBX_CLASS_CFG;
    ubx_packet.id = UBX_CFG_VALSET;  // CFG-VALSET command ID
    ubx_packet.len = 9;

    ubx_packet.payload.raw[0] = 0x00;   // Version
    ubx_packet.payload.raw[1] = 0x11;   // Layer = BBRAM (0x10) | RAM (0x01) see pg. 125 of Interface Description
    ubx_packet.payload.raw[2] = 0x00;   // Reserved
    ubx_packet.payload.raw[3] = 0x00;   // Reserved

    ubx_packet.payload.raw[4] = 0x02;   // LSB: Key ID (0x10720002)
    ubx_packet.payload.raw[5] = 0x00;   // CFG-I2COUTPROT-NMEA
    ubx_packet.payload.raw[6] = 0x72;   //
    ubx_packet.payload.raw[7] = 0x10;   // MSB: Key ID
    ubx_packet.payload.raw[8] = 0x00;   // Value = NMEA protocol (i.e. disable NMEA)

    return ublox_send_command(true);
}

ublox_status_e ublox_reset(void) {
    memset(&ubx_packet, 0, UBX_PACKET_LENGTH);

    ubx_packet.cls = UBX_CLASS_CFG;
    ubx_packet.id = 0x04;  // Reset command ID
    ubx_packet.len = 4;    // navBbrMask (X2) + resetMode (U1) + reserved (U1)

    // Set payload
    ubx_packet.payload.raw[0] = 0xFF;   // Clear all BBR sections
    ubx_packet.payload.raw[1] = 0xFF;   // Clear all BBR sections
    ubx_packet.payload.raw[2] = 0x00;   // Hardware reset
    ubx_packet.payload.raw[3] = 0x00;   // Reserved byte

    return ublox_send_command(false);  // Expect ACK
}
