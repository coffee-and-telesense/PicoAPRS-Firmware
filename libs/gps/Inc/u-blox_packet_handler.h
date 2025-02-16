/*******************************************************************************
 * @file: u-blox_packet_handler.h
 * @brief: Packet handling and validation for u-blox GNSS module
 ******************************************************************************/
#pragma once
#include "u-blox_types.h"
#include "u-blox_ubx_protocol.h"
#ifdef DEBUG
    #include "logging.h"
#endif
#include <string.h>

/*******************************************************************************
 * Public Function Prototypes
 ******************************************************************************/
ublox_status_e packet_prepare_command(ubx_packet_t *packet, uint8_t class, uint8_t id,
                                    const uint8_t *payload, uint16_t payload_len);
ublox_status_e packet_validate_response(const uint8_t *buffer, uint16_t size,
                                      uint8_t expected_class, uint8_t expected_id);
ublox_status_e packet_copy_payload(ubx_packet_t *packet, const uint8_t *buffer,
                                  uint16_t payload_len);

void calc_check_sum(const uint8_t *data, uint16_t len, uint8_t *ckA, uint8_t *ckB);
