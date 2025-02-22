#pragma once

#include "ubx_types.h"
#include "ubx_defs.h"
#include "gps_types.h"
#include "logging.h"


/**
 * @file ubx.h
 * @brief UBX protocol handling for u-blox GPS modules
 *
 * This header defines the interface for handling UBX protocol messages.
 * It provides functions for preparing UBX commands and validating responses
 * while working directly with provided buffers. The protocol layer is
 * stateless, with all state management handled by the driver layer.
 */

 #pragma once

 #include "ubx_types.h"
 #include "ubx_defs.h"
 #include "gps_types.h"

 /**
  * @brief Prepares a generic UBX command in the provided buffer
  *
  * Creates a UBX packet for commands that don't require payload data,
  * such as NAV_STATUS or NAV_PVT requests. The function handles sync
  * characters, message class/ID, and checksum calculation.
  *
  * @param buffer Buffer where the command will be formatted
  * @param cls UBX message class (e.g., UBX_CLASS_NAV)
  * @param id Message ID within the class (e.g., UBX_NAV_STATUS)
  * @return uint16_t Total size of the prepared packet in bytes, or 0 if error
  */
 uint16_t ubx_prepare_command(uint8_t* buffer, uint8_t cls, uint8_t id);

 /**
  * @brief Prepares a UBX configuration command in the provided buffer
  *
  * Creates a UBX-CFG-VALSET packet to configure module settings. This is
  * used for settings like enabling UBX protocol or disabling NMEA output.
  * The function handles all packet formatting including the configuration
  * payload structure.
  *
  * @param buffer Buffer where the command will be formatted
  * @param cfg_id Configuration item ID to set
  * @param value Value to set for the configuration
  * @return uint16_t Total size of the prepared packet in bytes, or 0 if error
  */
 uint16_t ubx_prepare_config_cmd(uint8_t* buffer, ubx_cfg_id_e cfg_id, uint8_t value);

 /**
  * @brief Validates a received UBX packet
  *
  * Checks that a received packet is valid and matches the expected type.
  * Validation includes:
  * - Sync characters
  * - Message class and ID
  * - Payload length
  * - Checksum
  *
  * @param buffer Buffer containing the received packet
  * @param size Size of the received data
  * @param expected_cls Expected message class
  * @param expected_id Expected message ID
  * @return gps_status_e GPS_OK if valid, appropriate error code otherwise
  */
 gps_status_e ubx_validate_packet(const uint8_t* buffer,
                                 uint16_t size,
                                 uint8_t expected_cls,
                                 uint8_t expected_id);

 /**
  * @brief Validates a received ACK/NACK response
  *
  * Special handling for ACK/NACK responses, which include verification that
  * the acknowledgment matches the command that was sent. This is crucial
  * for confirming configuration changes were accepted by the module.
  *
  * @param buffer Buffer containing the received packet
  * @param size Size of the received data
  * @param expected_cls Class of command being acknowledged
  * @param expected_id ID of command being acknowledged
  * @return gps_status_e GPS_OK if ACK received, GPS_ERROR_NACK if NACK received,
  *                      or other error code for invalid packets
  */
 gps_status_e ubx_validate_ack(const uint8_t* buffer,
                              uint16_t size,
                              uint8_t expected_cls,
                              uint8_t expected_id);

 /**
  * @brief Helper function to access frame structure from a buffer
  *
  * Provides type-safe access to the UBX frame structure overlaid on a
  * generic buffer. This allows easy access to frame fields while
  * maintaining proper alignment.
  *
  * @todo: Not currently used, but could be useful for future expansion
  *
  * @param buffer Pointer to buffer containing UBX data
  * @return ubx_frame_t* Pointer to frame structure
  */
 static inline ubx_frame_t* ubx_get_frame(uint8_t* buffer) {
     return (ubx_frame_t*)buffer;
 }

 /**
  * @brief Helper function to get total packet size for a given payload length
  *
  * Calculates the total packet size including header, payload, and checksum.
  * This is useful when allocating buffers or determining how many bytes to
  * transmit/receive.
  *
  * @param payload_len Length of the payload in bytes
  * @return uint16_t Total packet size in bytes
  */
 static inline uint16_t ubx_get_packet_size(uint16_t payload_len) {
     return UBX_HEADER_LENGTH + payload_len + UBX_CHECKSUM_LENGTH;
 }
