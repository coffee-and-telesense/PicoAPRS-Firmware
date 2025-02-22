
#pragma once
#include "ubx_messages.h"
#include "stdbool.h"
#include "string.h"

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
 *       If updating payload types, add a new structure in u-blox_packet_types.h then
 *       add it to the union here.
 */
typedef union {
    ubx_nav_status_s nav_status;
    ubx_nav_pvt_s nav_pvt;
    ubx_ack_ack_s ack_ack;
    uint8_t raw[256];
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
    UBX_FRAME_EMPTY,      // No data received yet
    UBX_FRAME_RECEIVED,   // Fresh data from device
    UBX_FRAME_IN_USE,     // Application is processing
    UBX_FRAME_PROCESSED   // Application is done with frame
} ubx_frame_state_e;

typedef struct {
    ubx_frame_t frame;
    ubx_frame_state_e frame_state;
    struct {
        uint8_t last_cls;
        uint8_t last_id;
    } state; // State variables for tracking last message sent
} ubx_protocol_t;
