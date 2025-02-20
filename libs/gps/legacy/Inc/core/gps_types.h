#pragma once
#include "error_types.h"

/**
 * @brief gps_types.h: Error and status codes for u-blox module
 *
 */
typedef enum {
    UBLOX_OK = ERR_OK, // No error
    UBLOX_ERROR = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x01, false),         // Generic error
    UBLOX_NOT_INITED = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x02, false),    // Module not initialized
    UBLOX_RETRY = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x03, false),         // Generic Retry command
    UBLOX_FRAME_IN_USE = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x04, false),  // Frame is currently in use
    // Communication Errors
    UBLOX_TIMEOUT = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x10, false),      // I2C/Communication timeout
    UBLOX_I2C_ERROR = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x11, true),     // I2C bus error (fatal)
    UBLOX_BUS_BUSY = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x12, false),     // Bus is busy
    // Data Validation Errorrs
    UBLOX_INVALID_DATA = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x30, false), // Received invalid/corrupted data
    UBLOX_NO_FIX = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x31, false),       // No GPS fix available
    UBLOX_INVALID_TIME = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x32, false), // Time validation failed
    UBLOX_BUFFER_OVERFLOW = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x33, true), // Buffer overflow (fatal)
    UBLOX_PACKET_NEEDS_PROCESSING = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x34, false) // Previous packet not processed
} gps_status_e;

/**
 * @brief Protocol types
 * @todo Add support for other protocols if needed
 */
typedef enum {
    UBX_PROTOCOL = 0,
    NMEA_PROTOCOL = 1
} gps_protocol_e;

/**
 * @brief Data types that can be requested from a protocol layer
 *      (e.g. position, time, velocity, etc). This creates type safety and
 *      extensibility for future data types.
 *
 */
typedef enum {
    GPS_DATA_POSITION,
    GPS_DATA_TIME,
    //GPS_DATA_VELOCITY, // Not implemented yet
    //GPS_DATA_HEADING,
    // Add more data types as needed
} gps_data_type_e;


/**
 * @brief Operation Modes of driver
 *
 */
typedef enum {
    GPS_MODE_BLOCKING,
    GPS_MODE_INTERRUPT
    // TODO: Could add GPS_MODE_DMA if needed
} gps_op_mode_e;

// Generic position structure that any protocol can fill
typedef struct {
    int32_t latitude;     // Degrees
    int32_t longitude;    // Degrees
    int32_t altitude;     // Millimeters above MSL
    uint8_t satellites;   // Number of satellites used
    uint8_t fix_type;    // Type of fix (none, 2D, 3D, etc)
    bool valid;          // Is this position valid?
} gps_position_t;

// Generic time structure
typedef struct {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    bool valid;
} gps_time_t;

// Add new structure for velocity
typedef struct {
    int32_t speed;         // Ground speed in m/s
    bool valid;
} gps_velocity_t;

// Add new structure for heading
/**
 * @todo: According to the integration manual head of vehicle is only valid if the receiver
 *       is in sensor fusion mode. I'm not sure yet what this means but see pg. 99 of the
 *       integration manual that mentions this.
 *
 *      Alternatively, we might get away with calculating the heading from the velocity vector;
 *      see the fields velN, velE, velD in the PVT message.
 */
typedef struct {
    int32_t heading;       // Heading in degrees
    bool valid;
} gps_heading_t;

/**
 * @brief Union of all possible gps data types, thereby allowing
 *        any data type to easily be passed around the system
 *
 */
// Union of all possible GPS data types
typedef union {
    gps_position_t position;
    gps_time_t     time;
    gps_velocity_t velocity;
    gps_heading_t  heading;
} gps_data_t;
