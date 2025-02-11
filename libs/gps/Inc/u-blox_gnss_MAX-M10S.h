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
#include "u-blox_Class_and_ID.h"
#include "u-blox_packet_types.h"
#ifdef DEBUG
  #include "logging.h"
#endif
#include "main.h"
#include "i2c.h"
#include <string.h>
#include <stdbool.h>

/*******************************************************************************
 * UBX Protocol Constants
 ******************************************************************************/
#define UBX_SYNC_CHAR_1         0xB5    /**< First sync character of UBX frame */
#define UBX_SYNC_CHAR_2         0x62    /**< Second sync character of UBX frame */
#define UBX_HEADER_LENGTH       6       /**< Header length: 2 sync + 1 class + 1 id + 2 length */
#define UBX_CHECKSUM_LENGTH     2       /**< Checksum length in bytes */
#define UBX_MAX_PAYLOAD_LENGTH  256     /**< Maximum payload length (see Integration Manual p.24) */
#define UBX_PACKET_LENGTH   (UBX_HEADER_LENGTH + UBX_MAX_PAYLOAD_LENGTH + UBX_CHECKSUM_LENGTH)


/* Payload Length Constants in Bytes*/
#define UBX_NAV_STATUS_LEN 16                /**< Length of UBX-NAV-STATUS payload */
#define UBX_ACK_ACK_LEN    2                 /**< Length of UBX-ACK-ACK payload */
#define UBX_CFG_VALSET_LEN 8                 /**< Length of UBX-CFG-VALSET payload */
#define UBX_NAV_PVT_LEN        92                /**< Length of UBX-NAV-PVT payload */

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/
/* Config Keys (32-bit) - See Interface Description p.124 */
#define UBX_CFG_L              0x01001000    /**< BBR layer configuration */
#define UBLOX_CFG_UBX_OUTPUT   0x10720001    /**< UBX protocol output config */

/* I2C Configuration */
#define UBLOX_I2C_ADDR         (0x42 << 1)   /**< Default u-blox I2C address (shifted for R/W bit) */
#define I2C_TIMEOUT            100           /**< I2C timeout in milliseconds */


/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * @brief Communication protocol type enumeration
 */
typedef enum {
    UBX = 0,     /**< UBX protocol (binary) */
    NMEA = 1     /**< NMEA protocol (not implemented) */
} ublox_comm_type_e;

/**
 * @brief Error and status codes for u-blox module
 *
 */
typedef enum {
    UBLOX_OK = 0,                       // Operation completed successfully
    UBLOX_ERROR = 1,                    // Generic error
    UBLOX_TIMEOUT = 2,                  // Operation timed out
    UBLOX_INVALID_DATA = 3,             // Received invalid or corrupted data
    UBLOX_NOT_INITIALIZED = 4,          // Module not initialized
    UBLOX_CHECKSUM_ERROR = 5,           // Message checksum verification failed
    UBLOX_PACKET_VALIDITY_ERROR = 6,    // Packet validation failed
    UBLOX_PACKET_NEEDS_PROCESSING = 7,  // Previous packet not yet processed
    UBLOX_NACK_ERROR = 8,               // Received NACK from module
} ublox_status_e;

/**
 * @brief Union of all possible UBX message payloads
 * @note This union allows for easy extensibility to add more payload types as needed.
 *       If updating payload types, add a new structure in u-blox_packet_types.h then
 *       add it to the union here.
 */
typedef union {
    ubx_nav_status_s nav_status;  // UBX-NAV-STATUS payload
    ubx_nav_pvt_s nav_pvt;        // UBX-NAV-PVT payload
    ubx_ack_ack_s ack_ack;        // UBX-ACK-ACK and UBX-ACK-NACK payloads
    // Add more payload types as needed
    uint8_t raw[256];             // UBX_MAX_PAYLOAD_LENGTH 256, some payloads might be larger and will need to adjust as necessary
} ubx_payload_t;


/**
 * @brief UBX packet structure, which included some additional fields for tracking packet data and validity
 *        of the frame.
 *
 * @note Careful management of this structure is important, as it is static and shared across the module.
 */
typedef struct {
  uint8_t sync1, sync2;
  uint8_t cls;
  uint8_t id;
  uint16_t len;                // Length of the payload-> does not include cls, id, or checksum bytes
  uint16_t counter;            // Keeps track of number of overall bytes received. Some responses are larger than 256 bytes.
  uint16_t startingSpot;       // The counter value needed to go past before we begin recording into payload array
  ubx_payload_t payload;       // Union of all payload types
  uint8_t checksumA;
  uint8_t checksumB;
  bool valid;                  // Valid bits for a current frame, needs to be cleared after each frame is processed
} ubx_packet_t;


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
