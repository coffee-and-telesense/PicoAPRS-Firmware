#pragma once

#include "target_config.h"
#include "u-blox_Class_and_ID.h"

// UBX Protocol Constants
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_HEADER_LENGTH 6  // 2 sync + 1 class + 1 id + 2 length
#define UBX_CHECKSUM_LENGTH 2
#define UBX_MAX_PAYLOAD_LENGTH 128  // Reduced for STM32L4KC RAM constraints
#define UBX_MAX_PACKET_LENGTH (UBX_HEADER_LENGTH + UBX_MAX_PAYLOAD_LENGTH + UBX_CHECKSUM_LENGTH)

// I2C Configuration for u-blox
#define UBLOX_I2C_ADDR 0x42  // Default u-blox I2C address
#define I2C_TIMEOUT 100      // Timeout in milliseconds

//use left shift as bit stuffing for I2C address tack on 0 for write and 1 for read
const uint8_t ublox_i2c_addr = 0x42 << 1; // u-blox i2c address

// Static buffer for payload data
static uint8_t ubx_payload_buffer[UBX_MAX_PAYLOAD_LENGTH];
static uint8_t ubx_packet_buffer[UBX_MAX_PACKET_LENGTH];





