#pragma once


#define MAX_BUFFER_SIZE 128
typedef enum {
    UBLOX_OK = 0x00,             // Operation successful
    UBLOX_ERROR = 0x01,          // General error
    UBLOX_INVALID_PARAM = 0x02,  // Invalid parameter provided
    UBLOX_TIMEOUT = 0x03,        // Operation timed out
    UBLOX_CHECKSUM_ERR = 0x04,   // Checksum verification failed
    UBLOX_I2C_ERROR = 0x05,      // I2C communication error (HAL error)
    // Add more error codes as needed
} gps_status_e;

// This struct allows for simple issuing of commands to the GPS device
// and allows for future extension of the command set
typedef enum {
    GPS_CMD_PVT, // We only have PVT for now
    // Example Extensions
    //GPS_CMD_CONFIG,
    //GPS_CMD_VERSION,
    //GPS_CMD_POWER_MODE,
    //Add other commands as needed
} gps_cmd_type_e;