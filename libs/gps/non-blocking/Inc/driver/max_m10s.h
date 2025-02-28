// gps/Inc/driver/max_m10s.h
#include "gps_types.h"
#include "ubx.h"
#include "hal_interface.h"
#include "logging.h"
#include "ubx_types.h"
// TODO: State/ case analysis

// What operation is the GPS driver currently performing?
typedef enum {
    GPS_INITING,
    GPS_READY,
    GPS_CONFIGURING,
    GPS_GETTING_DATA,
    GPS_RESETTING
} gps_state_e;

typedef enum {
    GPS_BLOCKING,
    GPS_NON_BLOCKING
} gps_bus_op_mode_e;

typedef enum {
    GPS_CONFIG_NONE,
    GPS_CONFIG_UBX_ENABLE,
    GPS_CONFIG_NMEA_DISABLE,
    GPS_CONFIG_DONE
} gps_config_stage_e;

// What bus operation is the GPS driver currently performing?
typedef enum {
    GPS_BUS_OP_NONE,
    GPS_BUS_OP_READ,
    GPS_BUS_OP_WRITE,
    GPS_BUS_OP_DELAY
} gps_bus_op_state_e;

// Flags for GPS driver
typedef union {
    uint16_t all;  // Access all flags at once, I would like to get this down to 8 bits if possible
    struct { // These are event flags that can be set or cleared to indicate state changes
        uint16_t inited : 1;        // Must specify type (uint16_t in this case)
        uint16_t read_done : 1;
        uint16_t write_done : 1;
        uint16_t delay_done : 1;    // Delay operation complete
        uint16_t read_error : 1;
        uint16_t write_error : 1;
        uint16_t bus_error : 1;
        uint16_t bus_busy : 1;
        uint16_t bus_timeout : 1;
        uint16_t config_valid : 1;  // Configuration validation
        uint16_t data_ready : 1;    // New data available
        uint16_t needs_reset : 1;   // Reset required flag
        uint16_t reserved : 4;
    } bits;
} gps_flags_t;


typedef struct {
    bool initialized;
    gps_state_e state;
    gps_bus_op_mode_e op_mode;
    gps_bus_op_state_e bus_state;
    gps_flags_t flags;
    gps_config_stage_e config_stage;
    gps_protocol_e protocol;      // Protocol layer
    void *interface;              // HAL interface
    uint8_t tx_buffer[UBX_MAX_PACKET_LENGTH];
    uint8_t rx_buffer[UBX_MAX_PACKET_LENGTH];
} max_m10s_t;

// Initialize with non-blocking support
gps_status_e max_m10s_init(max_m10s_t *dev, hal_interface_t *hal);

// Non-blocking operations
gps_status_e max_m10s_run(max_m10s_t *dev);  // State machine processing
