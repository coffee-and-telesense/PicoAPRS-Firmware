#pragma

#include "target_config.h"

// Sensor Specific Support Features
#define FEATURE_GNSS_UBLOX_MAX_M10S
#define FEATURE_I2C_SUPPORT

// Protocol Features
#ifdef FEATURE_GNSS_UBLOX_MAX_M10S
    #define FEATURE_UBX_PROTOCOL
    #define UBX_MAX_PAYLOAD_SIZE 256
#endif
