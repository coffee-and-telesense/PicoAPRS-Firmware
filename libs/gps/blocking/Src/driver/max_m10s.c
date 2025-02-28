
#include "max_m10s.h"

/*******************************************************************************
 * Private Type Definition
 ******************************************************************************/
static struct {
    bool initialized;
    gps_protocol_e protocol;
    ubx_protocol_t ubx; // UBX protocol packet
    // Could add NMEA packet storage here if needed
} gps_driver;

 /*******************************************************************************
  * Public Function Implementations
  ******************************************************************************/

 /**
  * @brief Initializes the u-blox GNSS module
  * @details Initializes I2C communication, clears packet structure, and configures
  *          module for UBX protocol output.
  * @return gps_status_e Initialization status
  */
 gps_status_e max_m10s_init(void){

    if (gps_driver.initialized) {
        return UBLOX_OK;    // Already initialized
    }

    // Initialize I2C Peripheral
    MX_I2C1_Init();

    // Initialize the driver structure
    memset(&gps_driver, 0, sizeof(gps_driver));

    // Setup protocol type
    gps_driver.protocol = UBX_PROTOCOL;
    // Configure ublox module for UBX protocol
    gps_status_e status = ubx_configure_i2c(&gps_driver.ubx);
    if(status != UBLOX_OK) {
        return status;
    }

    gps_driver.initialized = true;
    return UBLOX_OK;
 }

/**
  * @brief   Retrieves navigation status from the GNSS module
  * @details Requests NAV-STATUS message which provides receiver navigation status
  *          including fix type, validity flags, and time information
  * @return Returns UBLOX_OK if fix is valid and time is valid; else UBLOX_ERROR_XX
  */
gps_status_e max_m10s_get_nav_status(void) {
    if (!gps_driver.initialized) {
        return UBLOX_NOT_INITED;
    }

    if(gps_driver.ubx.frame_state == UBX_FRAME_IN_USE) {
        #ifdef DEBUG
            debug_print("Frame in use. If you need this data, call max_m10s_free_frame()\r\n");
        #endif
        return UBLOX_FRAME_IN_USE;
    }

    return ubx_get_nav_status(&gps_driver.ubx);
}

/**
 * @brief Get the current position from the GPS
 * @param position Pointer to position structure to fill
 * @return GPS_STATUS_OK if valid position obtained
 */
gps_status_e max_m10s_get_position(gps_position_t *position) {
    if (!position || !gps_driver.initialized) {
        return UBLOX_ERROR;
    }

    // Clear position structure
    memset(position, 0, sizeof(gps_position_t));

    gps_data_t data;
    gps_status_e status = ubx_extract_pvt_data(&gps_driver.ubx, GPS_DATA_POSITION, &data);
    if(status == UBLOX_OK) {
        *position = data.position;
    }
    return status;
}

/**
 * @brief Get the current time from the GPS
 * @param driver Pointer to driver structure
 * @param time Pointer to time structure to fill
 * @return GPS_STATUS_OK if valid time obtained
 */
gps_status_e max_m10s_get_time(gps_time_t *time) {
    if (!time || !gps_driver.initialized) {
        return UBLOX_ERROR;
    }

    // Clear time structure
    memset(time, 0, sizeof(gps_time_t));
    gps_data_t data;
    gps_status_e status = ubx_extract_pvt_data(&gps_driver.ubx, GPS_DATA_TIME, &data);
    if(status == UBLOX_OK) {
        *time = data.time;
    }
    return status;
}

/**
 * @brief Finished with current frame free it so protocol layer can reuse it.
 * @return
 */
gps_status_e max_m10s_free_frame(void) {
    if (!gps_driver.initialized) {
        return UBLOX_NOT_INITED;
    }
    return ubx_free_frame(&gps_driver.ubx);
}

gps_status_e max_m10s_reset(void) {
    if(!gps_driver.initialized) {
        return UBLOX_NOT_INITED;
    }
    return ubx_reset(&gps_driver.ubx);
 }
