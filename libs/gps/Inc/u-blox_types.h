// u-blox_types.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "error_types.h"
#include "u-blox_packet_types.h"

/**
 * @brief Error and status codes for u-blox module
 *
 */
typedef enum {
    UBLOX_OK = ERR_OK, // No error
    UBLOX_ERROR = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x01, false),         // Generic error
    UBLOX_NOT_INITED = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x02, false),    // Module not initialized
    UBLOX_RETRY = MAKE_ERROR(ERR_SUBSYSTEM_GPS, 0x03, false),         // Generic Retry command
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
} ublox_status_e;


/**
 * @brief Union of all possible UBX message payloads
 * @note This union allows for easy extensibility to add more payload types as needed.
 *       If updating payload types, add a new structure in u-blox_packet_types.h then
 *       add it to the union here.
 */
typedef union {
    ubx_nav_status_s nav_status;
    ubx_nav_pvt_s nav_pvt;
    ubx_ack_ack_s ack_ack;
    uint8_t raw[256];
} ubx_payload_t;

typedef struct {
    uint8_t sync1, sync2;
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    uint16_t counter;
    uint16_t startingSpot;
    ubx_payload_t payload;
    uint8_t checksumA;
    uint8_t checksumB;
    bool valid;
} ubx_packet_t;

typedef enum {
    UBX = 0,
    NMEA = 1
} ublox_comm_type_e;
