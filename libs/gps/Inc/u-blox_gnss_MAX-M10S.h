/*
File u-blox_gnss_MAX-M10S.h
*/
#pragma once

#include "target_config.h"
#include "u-blox_Class_and_ID.h"
#include "main.h"
#include "i2c.h"
#include <string.h>
#include <stdbool.h>

/*
General UBX Frame Description
*/
#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62
#define UBX_HEADER_LENGTH 6  // 2 sync + 1 class + 1 id + 2 length (bytes)
#define UBX_CHECKSUM_LENGTH 2
#define UBX_MAX_PAYLOAD_LENGTH 256  // Maximum payload length for u-blox (see pg. 24 of Integration Manual)
#define UBX_MAX_PACKET_LENGTH (UBX_HEADER_LENGTH + UBX_MAX_PAYLOAD_LENGTH + UBX_CHECKSUM_LENGTH)

/* 
Config Keys, each key consist of 32-bits
See pg 124 of Interface Description
We want to configure the UBX to be in BBR so it will retain this setting while battery backup supply is on. 
*/
#define UBX_CFG_L 0x01001000
#define UBLOX_CFG_UBX_OUTPUT 0x10720001

// I2C Configuration for u-blox
//use left shift as bit stuffing for I2C address tack on 0 for write and 1 for read
#define UBLOX_I2C_ADDR 0x42 << 1  // Default u-blox I2C address
#define I2C_TIMEOUT 100      // Timeout in milliseconds

// Static buffer for payload data
static uint8_t ubx_payload_buffer[UBX_MAX_PAYLOAD_LENGTH];
static uint8_t ubx_packet_buffer[UBX_MAX_PACKET_LENGTH];

typedef enum {
  UBX = 0, 
  NMEA = 1  //Not implemented
} comm_type_t;

// Union packet for holding current frame validity data
typedef struct{
    union {
        uint8_t all;
        struct {
            uint8_t check_sum_a : 1;    // Checksum A is valid if set
            uint8_t check_sum_b : 1;    // Checksum B is valid if set
            uint8_t class : 1;          // Class matches reqested class, valid if set
            uint8_t id : 1;             // ID matches requested ID, valid if set
            uint8_t packet : 1;         // Packet is valid if set, all other bits must be valid
        } bits;
    };
} valid_packet_t;


// This struct will hold the ubx frame data, it is a bidirectional packet used for sending and receiving
typedef struct {
  uint8_t cls;
  uint8_t id;
  uint16_t len;             // Length of the payload. Does not include cls, id, or checksum bytes
  uint16_t counter;         // Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
  uint16_t startingSpot;    // The counter value needed to go past before we begin recording into payload array
  uint8_t *payload;         // We will allocate RAM for the payload if/when needed.
  uint8_t checksumA;        // Given to us from module. Checked against the rolling calculated A/B checksums.
  uint8_t checksumB;  
  valid_packet_t valid;     // Valid bits for a current frame
} ubx_packet_t;


#define UBX_NAV_STATUS_LEN 16 

typedef struct
{
  uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
  uint8_t gpsFix; // GPSfix Type: 0x00 = no fix; 0x01 = dead reckoning only; 0x02 = 2D-fix; 0x03 = 3D-fix
                  // 0x04 = GPS + dead reckoning combined; 0x05 = Time only fix; 0x06..0xff = reserved
  union
  {
    uint8_t all;
    struct
    {
      uint8_t gpsFixOk : 1; // 1 = position and velocity valid and within DOP and ACC Masks.
      uint8_t diffSoln : 1; // 1 = differential corrections were applied
      uint8_t wknSet : 1;   // 1 = Week Number valid (see Time Validity section for details)
      uint8_t towSet : 1;   // 1 = Time of Week valid (see Time Validity section for details)
    } bits;
  } flags;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t diffCorr : 1;      // 1 = differential corrections available
      uint8_t carrSolnValid : 1; // 1 = valid carrSoln
      uint8_t reserved : 4;
      uint8_t mapMatching : 2; // map matching status: 00: none
                               // 01: valid but not used, i.e. map matching data was received, but was too old
                               // 10: valid and used, map matching data was applied
                               // 11: valid and used, map matching data was applied.
    } bits;
  } fixStat;
  union
  {
    uint8_t all;
    struct
    {
      uint8_t psmState : 2; // power save mode state
                            // 0: ACQUISITION [or when psm disabled]
                            // 1: TRACKING
                            // 2: POWER OPTIMIZED TRACKING
                            // 3: INACTIVE
      uint8_t reserved1 : 1;
      uint8_t spoofDetState : 2; // Spoofing detection state
                                 // 0: Unknown or deactivated
                                 // 1: No spoofing indicated
                                 // 2: Spoofing indicated
                                 // 3: Multiple spoofing indications
      uint8_t reserved2 : 1;
      uint8_t carrSoln : 2; // Carrier phase range solution status:
                            // 0: no carrier phase range solution
                            // 1: carrier phase range solution with floating ambiguities
                            // 2: carrier phase range solution with fixed ambiguities
    } bits;
  } flags2;
  uint32_t ttff; // Time to first fix (millisecond time tag): ms
  uint32_t msss; // Milliseconds since Startup / Reset: ms
} UBX_NAV_STATUS_data_t;

// FIXME: Calling function should use this structure for reading
//static UBX_NAV_STATUS_data_t packetUBX_NAV_STATUS; // Ram will be allocated when needed
static ubx_packet_t ubx_packet;

typedef enum {
    UBLOX_OK = 0,
    UBLOX_ERROR = 1,
    UBLOX_TIMEOUT = 2,
    UBLOX_INVALID_DATA = 3,
    UBLOX_NOT_INITIALIZED = 4,
    UBLOX_CHECKSUM_ERROR = 5,
    UBLOX_PACKET_VALIDITY_ERROR = 6,
    UBLOX_PACKET_NEEDS_PROCESSING = 7
    //TODO: Add more error codes as needed
} UBLOX_Status_t; 



// Function prototypes
UBLOX_Status_t ublox_init(void);
UBLOX_Status_t ublox_get_nav_status(void);
//static UBLOX_Status_t ublox_send_command(ubx_packet_t *outgoing_ubx, bool expect_ack_only); // TODO: Add parameters
//static void calc_check_sum(ubx_packet_t *outgoing_ubx);
//static uint16_t set_i2c_transaction_size(uint16_t len);


