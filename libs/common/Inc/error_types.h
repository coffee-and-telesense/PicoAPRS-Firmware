/****************************************************************************
 * @file: error_types.h
 * @brief: Common error handling system for PicoAPRS balloon platform
 *
 * Provides a unified error handling system for all components in the PicoAPRS
 * platform. Error codes are 16-bit values that encode:
 * - Whether the error is fatal
 * - Which subsystem generated the error
 * - Component-specific error code
 *
 * @note: Each subsystem should define its own specific error codes using the
 *       provided MAKE_ERROR macro.
 *
 ****************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

// Subsystem identifiers (bits 14-8, leaving MSB for fatal flag)
#define ERR_SUBSYSTEM_COMMON  0x00
#define ERR_SUBSYSTEM_GPS     0x01
#define ERR_SUBSYSTEM_RADIO   0x02
#define ERR_SUBSYSTEM_SENSORS 0x03
#define ERR_SUBSYSTEM_POWER   0x04
// ...Add more subsystems as needed...

// Masks for error code components
#define ERR_FATAL_MASK        0x8000  // Bit 15
#define ERR_SUBSYSTEM_MASK    0x7F00  // Bits 14-8
#define ERR_COMPONENT_MASK    0x00FF  // Bits 7-0

/**
 * @brief Error code format for pico balloon system
 *
 * 16-bit error code format: 0xFSCC where:
 * - F: Fatal flag (1 = fatal, 0 = non-fatal)
 * - S: Subsystem ID (7 bits)
 * - CC: Component-specific error code (8 bits)
 *
 * Example: 0x8102 = Fatal GPS error 02
 */

// Success code only - all other errors defined by specific components
#define ERR_OK 0x0000

// Helper Macros

#define IS_ERROR_FATAL(error) (((error) & ERR_FATAL_MASK) != 0)

#define GET_ERROR_SUBSYSTEM(error) (((error) & ERR_SUBSYSTEM_MASK) >> 8)

#define GET_ERROR_COMPONENT(error) ((error) & ERR_COMPONENT_MASK)

#define MAKE_ERROR(subsystem, component, fatal) \
    (((fatal) ? ERR_FATAL_MASK : 0) | \
    (((subsystem << 8) & ERR_SUBSYSTEM_MASK)) | \
    ((component) & ERR_COMPONENT_MASK))
