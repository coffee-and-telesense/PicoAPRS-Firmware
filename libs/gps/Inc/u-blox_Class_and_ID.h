/*******************************************************************************
 * @file: u-blox_Class_and_ID.h
 * @author: Reece Wayt
 * @date: January 13, 2025
 ******************************************************************************/
#pragma once

/*******************************************************************************
 * Class Message Types
********************************************************************************/
//UBX Message Class. See Chapter 3
#define UBX_CLASS_NAV 0x01  // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
#define UBX_CLASS_RXM 0x02  // Receiver Manager Messages: Satellite Status, RTC Status
#define UBX_CLASS_INF 0x04  // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
#define UBX_CLASS_ACK 0x05  // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
#define UBX_CLASS_CFG 0x06  // Configuration Input Messages: Configure the receiver.
#define UBX_CLASS_UPD 0x09  // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
#define UBX_CLASS_MON 0x0A  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
#define UBX_CLASS_AID 0x0B  //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
#define UBX_CLASS_TIM 0x0D  // Timing Messages: Time Pulse Output, Time Mark Results
#define UBX_CLASS_ESF 0x10  //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
#define UBX_CLASS_MGA 0x13  // Multiple GNSS Assistance Messages: Assistance data for various GNSS
#define UBX_CLASS_LOG 0x21  // Logging Messages: Log creation, deletion, info and retrieval
#define UBX_CLASS_SEC 0x27  // Security Feature Messages
#define UBX_CLASS_HNR 0x28  //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
//@TODO: NMEA messages are not implemented in this library
#define UBX_CLASS_NMEA 0xF0 // NMEA Strings: standard NMEA strings
#define UBX_CLASS_PUBX 0xF1 // Proprietary NMEA-format messages defined by u-blox


/*******************************************************************************
 * Message IDs
********************************************************************************/
// Class: NAV
#define UBX_NAV_STATUS 0x03    // Receiver Navigation Status
#define UBX_NAV_PVT 0x07       // Navigation Position Velocity Time Solution

// Class: CFG
#define UBX_CFG_VALSET 0x8A     // Set configuration item values
#define UBX_CFG_RST 0x04        // Reset Receiver / Clear Backup Data Structures

// Class: ACK
#define UBX_ACK_ACK 0x01        // Acknowledge
#define UBX_ACK_NACK 0x00       // Not Acknowledge

