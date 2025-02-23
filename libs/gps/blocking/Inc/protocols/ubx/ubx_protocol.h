/**
 * @file ubx_protocol.h*
 * @brief UBX Protocol Implementation
 * Frame Structure (see page 41 of interface manual):
 * +-------+-------+-------+-----+--------+---------+-------+-------+
 * | SYNC1 | SYNC2 | CLASS | ID  | LENGTH | PAYLOAD | CK_A  | CK_B  |
 * | 0xB5  | 0x62  |  1B   | 1B  |   2B   |   NB    |  1B   |  1B   |
 * +-------+-------+-------+-----+--------+---------+-------+-------+
 *
 * Total frame size  6 + N + 2 bytes (where N is payload length)
 * Checksum is calculated over the range: CLASS to PAYLOAD (inclusive)
 *
 * @note:The u-blox module supports two types of standards NMEA and UBX. NMEA is not implement in this library but could
 *       be implemented in the future.
 *
 */
#pragma once
#include "ubx_types.h"
#include "ubx_defs.h"
#include "ubx_messages.h"
#include "gps_types.h"
#include "ubx_packet_handler.h"
#include "stdint.h"
#include "i2c.h"
#include "logging.h"


/**
 * @brief Sends a UBX protocol command packet to the GPS module over I2C
 *
 * @details This function handles the complete process of sending a UBX command:
 *          1. Formats the packet according to UBX protocol specification
 *          2. Includes sync chars, class, ID, length, payload and checksum
 *          3. Transmits the formatted packet over I2C
 *          4. Implements required delays based on command type
 *          5. Optionally waits for and validates ACK response
 *
 * @param proto Pointer to UBX protocol structure containing command frame data
 * @param ack_only If true, function will wait for and parse ACK/NACK response
 *
 * @return UBLOX_OK if command sent successfully and (if ack_only) ACK received
 *         UBLOX_ERROR if I2C transmission fails or NACK received
 *
 * @note After sending a command, the function implements a delay to allow the GPS
 *       module to process it. Configuration commands require longer delays than
 *       standard polling commands.
 */
gps_status_e ubx_send_command(ubx_protocol_t proto, bool ack_only);

/**
 * @brief Frees the current frame in use
 *
 * @details The GPS driver maintains a single static frame for reading data from the GPS module.
 *          This function should be called after the data has been read from a frame.
 *
 * @param proto Pointer to UBX protocol structure
 *
 * @return UBLOX_OK if frame successfully freed
 */
gps_status_e ubx_free_frame(ubx_protocol_t* proto);

/**
 * @brief Extracts specific GPS data from a NAV-PVT message
 *
 * @details Handles the extraction of position, velocity, and time data from
 *          the NAV-PVT message format. Will request new data from the GPS
 *          module if current frame data is stale or empty.
 *
 * @param proto Pointer to UBX protocol structure containing frame data
 * @param type Type of data to extract (position or time)
 * @param data Pointer to GPS data structure where extracted data will be stored
 *
 * @return UBLOX_OK if data extracted successfully
 *         UBLOX_ERROR if invalid parameters
 *         UBLOX_FRAME_IN_USE if frame contains different message type
 *         Other error codes if new data request fails
 *
 * @note This function manages frame state and will only request new data from
 *       the GPS module when necessary to minimize I2C traffic
 */
gps_status_e ubx_extract_pvt_data(ubx_protocol_t* proto, gps_data_type_e type, gps_data_t* data);

/**
 * @brief Retrieves navigation status from the GNSS module
 *
 * @details Requests NAV-STATUS message which provides receiver navigation status
 *          including fix type, validity flags, and time information
 *
 * @param proto Pointer to UBX protocol structure
 *
 * @return UBLOX_OK if fix is valid and time is valid
 *         UBLOX_ERROR_XX if request fails
 */
gps_status_e ubx_get_nav_status(ubx_protocol_t* proto);

/**
 * @brief Configures the I2C output protocol settings of the u-blox module
 *
 * @details Sets up the module to use specified protocol (UBX) over I2C.
 *          Uses CFG-VALSET message to configure the I2COUTPROT setting.
 *          Configuration is stored in battery-backed RAM (BBRAM) to persist
 *          during power cycles while backup power is maintained.
 *
 * @param proto Pointer to UBX protocol structure
 *
 * @return UBLOX_OK if configuration successful
 *         UBLOX_ERROR_XX if configuration fails
 *
 * @note Configuration requires acknowledgment from module to confirm success
 */
gps_status_e ubx_configure_i2c(ubx_protocol_t* proto);

/**
 * @brief Resets the GPS module
 *
 * @details Sends a CFG-RST command to the GPS module to perform a reset.
 *          The reset type can be specified in the payload.
 *
 * @param proto Pointer to UBX protocol structure
 *
 * @return UBLOX_OK if reset successful
 *         UBLOX_ERROR_XX if reset fails
 */
gps_status_e ubx_reset(ubx_protocol_t* proto);
