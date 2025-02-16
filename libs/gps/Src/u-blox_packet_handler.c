/*******************************************************************************
 * @file: u-blox_packet_handler.c
 * @brief: Implementation of packet handling functions
 ******************************************************************************/
#include "u-blox_packet_handler.h"
#include "stdio.h"

ublox_status_e packet_prepare_command(ubx_packet_t *packet, uint8_t class, uint8_t id,
                                    const uint8_t *payload, uint16_t payload_len) {
    if (!packet || payload_len > UBX_MAX_PAYLOAD_LENGTH) {
        #ifdef DEBUG
            debug_print("Invalid packet or payload length\r\n");
        #endif
        return UBLOX_ERROR;
    }

    // Clear the packet structure
    memset(packet, 0, sizeof(ubx_packet_t));

    packet->cls = class;
    packet->id = id;
    packet->len = payload_len;
    packet->valid = false;

    if (payload && payload_len > 0) {
        memcpy(packet->payload.raw, payload, payload_len);
    }

    // Calculate checksum
    calc_check_sum((uint8_t*)&packet->cls, payload_len + 4,
                   &packet->checksumA, &packet->checksumB);

    return UBLOX_OK;
}

ublox_status_e packet_validate_response(const uint8_t *buffer, uint16_t size,
                                      uint8_t expected_class, uint8_t expected_id) {
    if (!buffer || size < UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH) {
        return UBLOX_INVALID_DATA;
    }

    // Verify sync chars
    if (buffer[0] != UBX_SYNC_CHAR_1 || buffer[1] != UBX_SYNC_CHAR_2) {
        return UBLOX_INVALID_DATA;
    }

    // Verify message class and ID
    if (buffer[2] != expected_class || buffer[3] != expected_id) {
        return UBLOX_INVALID_DATA;
    }
    // Get length from the message
    uint16_t payload_len = buffer[4] | (buffer[5] << 8);

    // Verify checksum
    uint8_t ckA = 0, ckB = 0;
    calc_check_sum(&buffer[2], payload_len + 4, &ckA, &ckB); // +4 for class, ID, length (2 bytes)

    if (ckA != buffer[size-2] || ckB != buffer[size-1]) {
        #ifdef DEBUG
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg), "Checksum mismatch: expected %02X:%02X, got %02X:%02X\r\n",
                     ckA, ckB, buffer[size-2], buffer[size-1]);
            debug_print(debug_msg);
        #endif
        return UBLOX_INVALID_DATA;
    }

    #ifdef DEBUG
        if(ckA == buffer[size-2] && ckB == buffer[size-1]){
            debug_print("Checksum OK\r\n");
        }
    #endif

    return UBLOX_OK;
}

ublox_status_e packet_copy_payload(ubx_packet_t *packet, const uint8_t *buffer,
                                  uint16_t payload_len) {
    if (!packet || !buffer || payload_len > UBX_MAX_PAYLOAD_LENGTH) {
        #ifdef DEBUG
            debug_print("Invalid packet or buffer\r\n");
        #endif
        return UBLOX_ERROR;
    }

    memcpy(&packet->payload.raw[0], &buffer[UBX_HEADER_LENGTH], payload_len);
    packet->valid = true;
    return UBLOX_OK;
}

/**
 * @brief Calculates Fletcher checksum for UBX packet
 *
 * @details Implements the Fletcher checksum algorithm for UBX protocol packets.
 *          Calculates checksums over class, ID, length, and payload bytes.
 *          The algorithm uses two running sums (checksumA and checksumB) where:
 *          - checksumA is a simple sum of bytes
 *          - checksumB is a sum of all checksumA values
 */
void calc_check_sum(const uint8_t *data, uint16_t len,
                          uint8_t *ckA, uint8_t *ckB) {
    *ckA = 0;
    *ckB = 0;

    for (uint16_t i = 0; i < len; i++) {
        *ckA += data[i];
        *ckB += *ckA;
    }
}
