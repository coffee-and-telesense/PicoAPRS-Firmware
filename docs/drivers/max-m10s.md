# u-blox MAX-M10S GNSS Driver API

## Overview
This is a lightweight driver that provides a C interface for communicating with the [u-blox MAX-M10S](https://www.u-blox.com/en/product/max-m10-series?legacy=Current#Documentation-&-resources) GNSS module over I2C. It supports configuration of the gps module to output UBX messages, NAV status checking for polling the device, and position data retrieval using the UBX protocol.

The driver is designed for use with STM32 microcontrollers and **depends on the HAL I2C driver for communication.**

**Note**: Communication with the device is realtively slow and requires a 1 second delay between reading and writing to the device. I was not 
able to find information in the datasheet that specified this for I2C communication but I arrived at 1 second through trial and error. Anyhting less then 1 second tended to cause errors and the Ublox module would not respond with a valid message. Because of this you will see `HAL_Delay(1000)` in the driver code's method for sending a ubx command.

## Key Features
- I2C communication with the MAX-M10S module
- UBX protocol support
- Navigation status checking
- Position, Velocity, and Time (PVT) data retrieval
- Module configuration and reset capabilities

## Getting Started
1. Initialize the module using `ublox_init()`
2. Check for valid GPS fix using `ublox_get_nav_status()`
3. Once a fix is obtained, retrieve position data using `ublox_get_pvt()` followed by `ublox_get_curr_position()`

## Example Usage
```c
int32_t lat, lon, height;

// Initialize the module
if (ublox_init() != UBLOX_OK) {
    // Handle error
}

// Wait for valid fix
while (ublox_get_nav_status() != UBLOX_OK) {
    HAL_Delay(1000);  // Check every second
}

// Get position data
if (ublox_get_pvt() == UBLOX_OK) {
    if (ublox_get_curr_position(&lat, &lon, &height) == UBLOX_OK) {
        // Process position data
        // Note: lat/lon are in degrees * 10^-7
        // height is in millimeters
    }
}
```

## Function Documentation

### `ublox_init()`
Initializes the u-blox GNSS module.

This function performs the following initialization steps:
1. Initializes the I2C peripheral
2. Configures the module for UBX protocol output
3. Disables NMEA protocol output

**Returns:**
- `UBLOX_OK` if initialization successful, error code otherwise

### `ublox_get_nav_status()`
Retrieves navigation status from the GNSS module.

Requests and processes the UBX-NAV-STATUS message to determine if:
- GPS has a valid fix
- Time data is valid

**Returns:**
- `UBLOX_OK` if valid fix and time data available, error code otherwise

### `ublox_get_pvt()`
Requests Position, Velocity, and Time data from the module.

Sends a UBX-NAV-PVT message request to the module. This function must be called before `ublox_get_curr_position()` to get fresh position data.

**Returns:**
- `UBLOX_OK` if valid PVT data received, error code otherwise

### `ublox_get_curr_position()`
Retrieves the current position from the most recent PVT data.

**Note:** This function must be called after a successful `ublox_get_pvt()` call

**Parameters:**
- `lat`: Pointer to store latitude (degrees * 10^-7)
- `lon`: Pointer to store longitude (degrees * 10^-7)
- `height`: Pointer to store height above mean sea level (millimeters)

**Returns:**
- `UBLOX_OK` if valid position data retrieved, error code otherwise

**Example coordinate conversion:**
- Latitude: 455137510 * 10^-7 = 45.5137510° N
- Longitude: -1226464697 * 10^-7 = -122.6464697° W
- Height: 36673 mm = 36.673 meters

### `ublox_reset()`
Resets the u-blox GNSS module.

Performs a hardware reset of the module and clears all configuration data in RAM and battery-backed RAM.

**Returns:**
- `UBLOX_OK` if reset successful, error code otherwise

## Status Codes
```c
typedef enum {
    UBLOX_OK = 0,                    // Operation completed successfully
    UBLOX_ERROR = 1,                 // Generic error
    UBLOX_TIMEOUT = 2,               // Operation timed out
    UBLOX_INVALID_DATA = 3,          // Received invalid or corrupted data
    UBLOX_NOT_INITIALIZED = 4,        // Module not initialized
    UBLOX_CHECKSUM_ERROR = 5,        // Message checksum verification failed
    UBLOX_PACKET_VALIDITY_ERROR = 6,  // Packet validation failed
    UBLOX_PACKET_NEEDS_PROCESSING = 7, // Previous packet not yet processed
    UBLOX_NACK_ERROR = 8,            // Received NACK from module
} ublox_status_e;
```

## Test Application
The driver includes a test application that demonstrates proper usage of this API.
The test application provides these commands:

- `I`: Initialize the GPS module
- `F`: Check GPS Fix status
- `L`: Get Location data (lat/lon/height)

The test application shows proper initialization sequence, error handling, and data retrieval methods. It also demonstrates how to convert the raw position values into human-readable format.

See `u-blox_test.c` for the complete test application implementation.

**Compiling and building the test application**
```bash
cd tests/test_gps
cmake --preset Debug # If using cmake presets and vscode extension
cmake --build build/Debug/
```
**Flashing the target board**   
I'm using the nucleo-l432kc board but this application could easily be extended to other boards with the proper 
linking of the stm32 HAL for your target board

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

