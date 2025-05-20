#pragma once
#include "ubx_messages.h"
#include "ubx_defs.h"
#include "stdbool.h"
#include "string.h"
#include "stdint.h"

/*******************************************************************************
 * @file: ubx_types.h
 * @brief: UBX protocol types and structures for u-blox GNSS modules
 *
 * @note: Contains all protocol-specific types and structures including:
 *        - Packet structure
 *        - Payload structures
 ******************************************************************************/


/**
 * @brief Union of all possible UBX message payloads
 * @note This union allows for easy extensibility to add more payload types as needed.
 *       If updating payload types, add a new structure in ubx_messages.h then
 *       add it to the union here.
 */
typedef union __attribute__((packed)) {
    ubx_nav_status_s nav_status;
    ubx_nav_pvt_s nav_pvt;
    ubx_ack_ack_s ack_ack;
    uint8_t raw[UBX_MAX_PAYLOAD_LENGTH]; // Raw payload for generic handling
} ubx_payload_t;

// Basic UBX packet structure - only contains frame-specific fields
typedef struct __attribute__((packed)) {
    uint8_t sync1, sync2;
    uint8_t cls;
    uint8_t id;
    uint16_t len;
    ubx_payload_t payload;
    uint8_t checksumA;
    uint8_t checksumB;
} ubx_frame_t;


typedef enum {
    UBX_CFG_I2C_UBX_ENABLE =    0x10720001,   // CFG-I2COUTPROT-UBX
    UBX_CFG_I2C_NMEA_DISABLE =  0x10720002,   // CFG-I2COUTPROT-NMEA
    UBX_CFG_RATE_MEAS =         0x30210001,   // CFG-RATE-MEAS
    // Add other config IDs as needed
} ubx_cfg_id_e;
