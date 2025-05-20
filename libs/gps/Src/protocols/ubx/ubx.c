// ubx_protocol.c
#include "ubx.h"
#include <string.h>
#include <stdio.h>

#ifdef DEBUG
    #define debug_print(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
    #define debug_print(fmt, ...) ((void)0) //no operations
#endif  

/**
 * @brief Calculate Fletcher checksum for UBX packet. Checksum is calculated from
 *          class, ID, length, and payload bytes in the UBX protocol.
 * @param data - Starting byte to calculate checksum from
 * @param len - Number of bytes to include in checksum
 * @param ckA - Pointer to store first checksum byte
 * @param ckB - Pointer to store second checksum byte
 */
static void calc_checksum(const uint8_t *data, uint16_t len, uint8_t *ckA, uint8_t *ckB) {
    *ckA = 0;
    *ckB = 0;

    for (uint16_t i = 0; i < len; i++) {
        *ckA += data[i];
        *ckB += *ckA;
    }
}


uint16_t ubx_prepare_command(uint8_t* buffer, uint8_t cls, uint8_t id) {
    if (!buffer) {
        return 0;
    }

    ubx_frame_t* frame = (ubx_frame_t*)buffer;

    // Clear frame memory to avoid garbage data
    memset(frame, 0, sizeof(ubx_frame_t));

    // Set header
    frame->sync1 = UBX_SYNC_CHAR_1;
    frame->sync2 = UBX_SYNC_CHAR_2;
    frame->cls = cls;
    frame->id = id;
    frame->len = 0;  // No payload for basic commands

    // Calculate checksum starting from class byte
    uint8_t checksumA, checksumB;
    calc_checksum(&frame->cls, frame->len + 4, &checksumA, &checksumB);

    // After calculating checksum we need to ensure the checksum comes right after the payload
    // base adress of frame + header + payload = adress of checksum
    frame->payload.raw[frame->len] = checksumA;
    frame->payload.raw[frame->len + 1] = checksumB;  // Checksum is 2 bytes long

    // Return total packet size
    return UBX_HEADER_LENGTH + frame->len + UBX_CHECKSUM_LENGTH;
}

/** */
uint16_t ubx_prepare_config_cmd_by_size(uint8_t* buffer, ubx_cfg_id_e cfg_id, const void* value_ptr, uint8_t value_size) {
    if (!buffer || !value_ptr || value_size > 4) {
        return 0;
    }

    ubx_frame_t* frame = (ubx_frame_t*)buffer;

    // Clear frame memory to avoid garbage data
    memset(frame, 0, sizeof(ubx_frame_t));

    // Set header
    frame->sync1 = UBX_SYNC_CHAR_1;
    frame->sync2 = UBX_SYNC_CHAR_2;
    frame->cls = UBX_CLASS_CFG;
    frame->id = UBX_CFG_VALSET;

    // Prepare config payload
    uint8_t* payload = frame->payload.raw;
    payload[0] = 0x00;  // Version
    payload[1] = UBX_CFG_LAYER_RAM | UBX_CFG_LAYER_BBR;  // RAM + BBR
    payload[2] = 0x00;  // Reserved
    payload[3] = 0x00;  // Reserved

    // Set Key ID (little endian)
    payload[4] = (cfg_id >> 0)  & 0xFF;
    payload[5] = (cfg_id >> 8)  & 0xFF;
    payload[6] = (cfg_id >> 16) & 0xFF;
    payload[7] = (cfg_id >> 24) & 0xFF;

    // Set value with proper size (little endian)
    memcpy(&payload[8], value_ptr, value_size);

    frame->len = 8 + value_size;  // 8 bytes for header + value size

    // Calculate checksum starting from class byte
    uint8_t checksumA, checksumB;
    calc_checksum(&frame->cls, frame->len + 4, &checksumA, &checksumB);

    // Set checksum after payload
    frame->payload.raw[frame->len] = checksumA;
    frame->payload.raw[frame->len + 1] = checksumB;

    // Return total packet size
    return UBX_HEADER_LENGTH + frame->len + UBX_CHECKSUM_LENGTH;
}

/* Wrapper functions for specific value sizes */
uint16_t ubx_prepare_config_cmd_u8(uint8_t* buffer, ubx_cfg_id_e cfg_id, uint8_t value) {
    return ubx_prepare_config_cmd_by_size(buffer, cfg_id, &value, sizeof(uint8_t));
}

uint16_t ubx_prepare_config_cmd_u16(uint8_t* buffer, ubx_cfg_id_e cfg_id, uint16_t value) {
    return ubx_prepare_config_cmd_by_size(buffer, cfg_id, &value, sizeof(uint16_t));
}

uint16_t ubx_prepare_config_cmd_u32(uint8_t* buffer, ubx_cfg_id_e cfg_id, uint32_t value) {
    return ubx_prepare_config_cmd_by_size(buffer, cfg_id, &value, sizeof(uint32_t));
}


// Method is deprecated but keeping for now. 
uint16_t ubx_prepare_config_cmd(uint8_t* buffer, ubx_cfg_id_e cfg_id, uint8_t value) {
    if (!buffer) {
        return 0;
    }

    ubx_frame_t* frame = (ubx_frame_t*)buffer;

    // Clear frame memory to avoid garbage data
    memset(frame, 0, sizeof(ubx_frame_t));

    // Set header
    frame->sync1 = UBX_SYNC_CHAR_1;
    frame->sync2 = UBX_SYNC_CHAR_2;
    frame->cls = UBX_CLASS_CFG;
    frame->id = UBX_CFG_VALSET;

    // Prepare config payload
    uint8_t* payload = frame->payload.raw;
    payload[0] = 0x00;  // Version
    payload[1] = UBX_CFG_LAYER_RAM | UBX_CFG_LAYER_BBR;  // RAM + BBR
    payload[2] = 0x00;  // Reserved
    payload[3] = 0x00;  // Reserved

    // Set Key ID (little endian)
    payload[4] = (cfg_id >> 0)  & 0xFF;
    payload[5] = (cfg_id >> 8)  & 0xFF;
    payload[6] = (cfg_id >> 16) & 0xFF;
    payload[7] = (cfg_id >> 24) & 0xFF;

    // Set value
    payload[8] = value;

    frame->len = 9;  // Fixed length for config messages

    // Calculate checksum starting from class byte
    uint8_t checksumA, checksumB;
    calc_checksum(&frame->cls, frame->len + 4, &checksumA, &checksumB);

    // After calculating checksum we need to ensure the checksum comes right after the payload
    // base adress of frame + header + payload = adress of checksum
    frame->payload.raw[frame->len] = checksumA;
    frame->payload.raw[frame->len + 1] = checksumB;  // Checksum is 2 bytes long

    // Return total packet size
    return UBX_HEADER_LENGTH + frame->len + UBX_CHECKSUM_LENGTH;
}


gps_status_e ubx_validate_packet(const uint8_t* buffer, uint16_t size,
    uint8_t expected_cls, uint8_t expected_id) {
    if (!buffer || size < UBX_HEADER_LENGTH + UBX_CHECKSUM_LENGTH) {
        return UBLOX_INVALID_PARAM;
    }

    const ubx_frame_t* frame = (const ubx_frame_t*)buffer;

    // Check sync chars
    if (frame->sync1 != UBX_SYNC_CHAR_1 || frame->sync2 != UBX_SYNC_CHAR_2) {
        return UBLOX_ERROR;
    }

    // Check class and ID
    if (frame->cls != expected_cls || frame->id != expected_id) {
        return UBLOX_ERROR;
    }

    // Validate length
    if (frame->len > UBX_MAX_PAYLOAD_LENGTH) {
        return UBLOX_ERROR;
    }

    // Verify total packet size matches expected
    if (size != UBX_HEADER_LENGTH + frame->len + UBX_CHECKSUM_LENGTH) {
        return UBLOX_ERROR;
    }

    // Validate checksum
    uint8_t ckA = 0, ckB = 0;
    calc_checksum(&frame->cls, frame->len + 4, &ckA, &ckB);

    if (ckA != buffer[size-2] || ckB != buffer[size-1]) {
        return UBLOX_ERROR;
    }

    return UBLOX_OK;
}

gps_status_e ubx_validate_ack(const uint8_t* buffer,
    uint16_t size,
    uint8_t expected_cls,
    uint8_t expected_id) {
        gps_status_e status = ubx_validate_packet(buffer, size, UBX_CLASS_ACK, UBX_ACK_ACK);
        if (status != UBLOX_OK) {
            if(buffer[3] == UBX_ACK_NACK) {
                #ifdef DEBUG
                    debug_print("Received NACK response\r\n");
                #endif
                return UBLOX_ERROR;
            }
            else {
                #ifdef DEBUG
                    debug_print("Invalid response\r\n");
                #endif
                return status;
            }
            return status;
        }
        if(buffer[6] != expected_cls || buffer[7] != expected_id) {
            #ifdef DEBUG
                debug_print("ACK payload does not match sent command\r\n");
            #endif
            return UBLOX_ERROR;
        }
        return UBLOX_OK;

    }
