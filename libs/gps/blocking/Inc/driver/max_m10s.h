/*******************************************************************************
 * @file: max_m10s.h
 * @brief: Implementation of the u-blox MAX-M10S GNSS module driver using UBX protocol
 *         Provides initialization and communication functions for configuring and
 *         retrieving data from the GPS module over I2C.
 *
 * @note: This implementation focuses on UBX protocol support and I2C communication.
 *        NMEA protocol support may be added in future versions.
 *
 * @version: 1.1
 * @sources:
 *   - u-blox MAX-M10 Interface Manual v5.10 (page of this manual will be referenced in comments)
 *     https://www.u-blox.com/en/product/max-m10-series#Documentation-&-resources
 *   - SparkFun u-blox GNSS Arduino Library v3
 *     https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3
 *
 * @author: Reece Wayt
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
 * @return GPS_STATUS_OK if successful
 */
gps_status_e max_m10s_init(void);

/**
 * @brief Get the current position from the GPS
 * @param position- Pointer to position structure to fill
 * @return GPS_STATUS_OK if valid position obtained
 */
gps_status_e max_m10s_get_position(gps_position_t *position);

/**
 * @brief Get the current time from the GPS
 * @param time - Pointer to time structure to fill
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
 * @brief Free the current frame in use. The gps driver maintains a single static frame
 *       for reading data from the GPS module. This function should be called after the
 *      data has been read from a frame.
 */
gps_status_e max_m10s_free_frame(void);

/**
 * @brief Reset the GPS module
 * @return GPS_STATUS_OK if reset successful
 */
gps_status_e max_m10s_reset(void);
