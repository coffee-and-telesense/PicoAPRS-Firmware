/*******************************************************************************
 * @file: u-blox_gnss_MAX-M10s.h
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
#include "u-blox_types.h"
#include "u-blox_ubx_protocol.h"
#include "u-blox_packet_types.h"
#include "u-blox_packet_handler.h"
#ifdef DEBUG
  #include "logging.h"
  #include "stdio.h"
#endif
#include "main.h"
#include "i2c.h"
#include <string.h>
#include <stdbool.h>



/*******************************************************************************
 * Global Variables
 ******************************************************************************/
//Add global variables as needed

/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/
/**
 * @brief Initializes the u-blox GNSS module
 * @return ublox_status_e Initialization status
 */
ublox_status_e ublox_init(void);

/**
 * @brief Retrieves navigation status from the GNSS module
 * @return ublox_status_e Status of the navigation data request
 */
ublox_status_e ublox_get_nav_status(void);

/**
 * @brief Sends message to ublox module to get position, velocity, and time (PVT) data from the GNSS module
 */
ublox_status_e ublox_get_pvt(void);

/**
 * @brief Read the most recent pvt data
 */
ublox_status_e ublox_get_curr_position(int32_t *lat, int32_t *lon, int32_t *height);

/**
 * @brief Resets the u-blox GNSS module which will clear all configurations and data structures in RAM
 *        and BBR memory. Hence, resets everything to default settings.
 * @return ublox_status_e Reset status
 */
ublox_status_e ublox_reset(void);
