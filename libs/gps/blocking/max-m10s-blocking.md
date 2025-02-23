# u-blox MAX-M10S GNSS Blocking Driver API

## Overview
This is a lightweight driver that provides a C interface for communicating with the [u-blox MAX-M10S](https://www.u-blox.com/en/product/max-m10-series?legacy=Current#Documentation-&-resources) GNSS module over I2C. It supports configuration of the gps module to output UBX messages, NAV status checking for polling the device, and position data retrieval using the UBX protocol.

**Note**: All operations of this driver are blocking, which means it uses the respective I2C Hal blocking API functions. Because of this, communication with the device is realtively slow and requires a 1 second delay between reading and writing to the device. I was not  able to find information in the datasheet that specified this for I2C communication but I arrived at 1 second through trial and error. Anything less then 1 second tended to cause errors and the ublox module would not respond with a valid message. Because of this you will see `HAL_Delay(1000)` in the driver code's method for sending a ubx command.

## Library Structure
```bash
PicoAPRS-Firmware\libs\gps\blocking
❯ tree
.
|-- CMakeLists.txt                  # Build configuration
|-- Inc
|   |-- core                        # Core definitions shared across layers
|   |   `-- gps_types.h             # Common GPS data structures and types
|   |-- driver                      # Driver API layer headers
|   |   `-- max_m10s.h              # Public driver interface
|   `-- protocols                   # Protocol-specific headers
|       `-- ubx                     # UBX protocol implementation
|           |-- ubx_defs.h          # UBX protocol constants and definitions
|           |-- ubx_messages.h      # UBX message structure definitions
|           |-- ubx_packet_handler.h # Low-level packet operations
|           |-- ubx_protocol.h      # UBX protocol interface
|           `-- ubx_types.h         # UBX-specific type definitions
`-- Src
   |-- driver                       # Driver API implementation
   |   `-- max_m10s.c               # Driver interface implementation
   `-- protocols                    # Protocol implementations
       `-- ubx                      # UBX protocol source
           |-- ubx_packet_handler.c # Packet handling implementation
           `-- ubx_protocol.c       # UBX protocol implementation
```
## Driver Architecture
The driver is organized into two distinct layers to provide a simple API interface that abstracts away from protocol implementations.
### 1. API Layer
`max_m10s.h/.c`
- This is a simple, protocol agnostic interface that contains common GPS operations
- It does not have direct Hardware access

### 2. Protocol Layer
`ubx_protocol.h/.c`
- Responsible for UBX frame formatting and parse
- Performs data validation and check sums
- This layer maintains a **single ubx frame** that represents the current GPS data.
- Manages I2C communication

**Future Iterations** should move away from the use of global variables found in this library. At the time of writing this this library is simply a prototype and is not considered production code.

**UBX Frame Lifecycle**: Since there is only one frame the application code is responsible for freeing the frame once done processing (see the function `max_m10s_free_frame()`). States are managed through the `ubx_frame_state_e` enum.

```c
// core/gps_types.h
typedef enum {
    UBX_FRAME_EMPTY,      // No data received yet
    UBX_FRAME_RECEIVED,   // Fresh data from device
    UBX_FRAME_IN_USE,     // Application is processing
    UBX_FRAME_PROCESSED   // Application is done with frame
} ubx_frame_state_e;
```


## Example Usage
```c
#include "max_m10s.h"

gps_data_t gps_data;

// Initialize the module
if (max_m10s_init() != UBLOX_OK) {
    // Handle error
}

// Wait for valid fix
if (max_m10s_get_nav_status() != UBLOX_OK) {
    // This is a useful method if want to poll when a nav fix is ready or
    // after a system reset since the get pvt data frame is much larger and
    // more involved
}

// Get position data
if (max_m10s_get_position(&gps_data.position) == UBLOX_OK) {
    if (position.valid) {
        // Process position data
        // Note: latitude/longitude are in degrees * 10^-7
        // altitude is in millimeters
    }
    if(max_m10s_get_time(&gps_data.time)) {
        // Process time data
    }
}

// Mark frame as processed when done
max_m10s_free_frame();
```

## Function Documentation
### `max_m10s_get_position()`
Retrieves the current position data from the GPS.

**Parameters:**
- `position`: Pointer to position structure to fill with data

**Returns:**
- `UBLOX_OK` if valid position data retrieved, error code otherwise

### `max_m10s_get_time()`
Retrieves the current UTC time from the GPS.

**Parameters:**
- `time`: Pointer to time structure to fill with data

**Returns:**
- `UBLOX_OK` if valid time data retrieved, error code otherwise

### `max_m10s_free_frame()`
Marks the current frame as processed, indicating the application is done using the data.

**Returns:**
- `UBLOX_OK` if successful, error code otherwise



## Test Application
The driver includes a test application that demonstrates proper usage of this API and
was used in verifying API methods


**Compiling and building the test application**
```bash
cd tests/test_gps
cmake --preset Debug # If using cmake presets and vscode extension
cmake --build build/Debug/
```
**Flashing the target board**
I'm using the nucleo-l432kc board but this application could easily be extended to other boards with the proper linking of the stm32 HAL for your target board

You can flash the board using STM32CubeProgrammer CLI directly, or automate this step using VS Code Tasks. I've provided the tasks.json configuration below.
```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "type": "shell",
            "label": "Flash Test GPS",
            "command": "STM32_Programmer_CLI",
            "args": [
                "--connect",
                "port=swd",
                "--download",
                "${workspaceFolder}/tests/test_gps/build/Debug/test_gps.elf",
                "-hardRst",
                "-rst",
                "--start"
            ],
            "options": {
                "cwd": "${workspaceFolder}"
            },
            "problemMatcher": []
        }
    ]
}
```
