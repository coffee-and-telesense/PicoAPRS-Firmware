/*******************************************************************************
 * @file: .h
 * @brief: U-blox (UBX) protocol frame structure and message definitions for MAX-M10
 *
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
 * @version: 1.0
 * @sources:
 *   - u-blox MAX-M10 Interface Manual v5.10 (page of this manual will be referenced in comments)
 *     https://www.u-blox.com/en/product/max-m10-series#Documentation-&-resources
 *   - SparkFun u-blox GNSS Arduino Library v3
 *     https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3
 *
 * @author: Reece Wayt
 * @date: January 13, 2025
 ******************************************************************************/
#pragma once

#include "target_config.h"
#include "error_types.h"
#include "gps_types.h"
#include "ubx_defs.h"
#include "ubx_packet_handler.h"
#include "ubx_messages.h"
#include "ubx_protocol.h"

#ifdef DEBUG
  #include "logging.h"
#endif
#include "main.h"
#include "i2c.h"
#include <string.h>
#include <stdbool.h>


/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/
/**
 * @brief Initialize the MAX-M10S GPS module
 * @param driver Pointer to driver structure
 * @return GPS_STATUS_OK if successful
 */
gps_status_e max_m10s_init(void);

/**
 * @brief Get the current position from the GPS
 * @param driver Pointer to driver structure
 * @param position Pointer to position structure to fill
 * @return GPS_STATUS_OK if valid position obtained
 */
gps_status_e max_m10s_get_position(gps_position_t *position);

/**
 * @brief Get the current time from the GPS
 * @param driver Pointer to driver structure
 * @param time Pointer to time structure to fill
 * @return GPS_STATUS_OK if valid time obtained
 */
gps_status_e max_m10s_get_time(gps_time_t *time);

/**
  * @brief Retrieves navigation status from the GNSS module
  * @details Requests NAV-STATUS message which provides receiver navigation status
  *          including fix type, validity flags, and time information
  * @return Returns UBLOX_OK if fix is valid and time is valid; else UBLOX_ERROR_XX
  */
 gps_status_e max_m10s_get_nav_status(void);

/**
 * @brief Mark current frame as processed
 *
 */
gps_status_e max_m10s_free_frame(void);

/**
 * @brief Reset the GPS module
 * @param driver Pointer to driver structure
 * @return GPS_STATUS_OK if reset successful
 */
gps_status_e max_m10s_reset(void);

/**
 * @brief Get the GPS interface structure for this driver
 * @param driver Pointer to driver structure
 * @return Filled gps_interface_t structure
 */
// TODO: Implement this function
//gps_interface_t max_m10s_get_interface(max_m10s_driver_t *driver);
