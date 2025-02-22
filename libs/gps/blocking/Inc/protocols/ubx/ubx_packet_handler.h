/*******************************************************************************
 * @file: u-blox_packet_handler.h
 * @brief: Packet handling and validation for u-blox GNSS module
 ******************************************************************************/
#pragma once
#include "ubx_types.h"
#include "ubx_defs.h"
#include "gps_types.h"
#ifdef DEBUG
    #include "logging.h"
#endif
#include <string.h>

/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/
gps_status_e packet_prepare_command(ubx_frame_t *packet, uint8_t class, uint8_t id,
                                    const uint8_t *payload, uint16_t payload_len);
gps_status_e packet_validate_response(const uint8_t *buffer, uint16_t size,
                                      uint8_t expected_class, uint8_t expected_id);
gps_status_e packet_copy_payload(ubx_frame_t *packet, const uint8_t *buffer,
                                  uint16_t payload_len);


/**
 * @brief Validates an ACK/NACK response packet
 * @param buffer Pointer to received data buffer
 * @param size Length of received data
 * @param expected_cls Expected class ID from sent command
 * @param expected_id Expected message ID from sent command
 * @return gps_status_e UBLOX_OK if valid ACK received, UBLOX_ERROR if NACK or invalid
 */
gps_status_e packet_validate_ack(const uint8_t *buffer, uint16_t size,
                                uint8_t expected_cls, uint8_t expected_id);

void calc_check_sum(ubx_frame_t* packet);
