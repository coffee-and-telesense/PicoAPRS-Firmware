/*******************************************************************************
 * @file: ubx_protocol.h
 * @brief: U-blox (UBX) protocol frame structure and message definitions for MAX-M10
 * 
 * Frame Structure (see page 41 of interface manual):
 * +-------+-------+-------+-----+--------+---------+-------+-------+
 * | SYNC1 | SYNC2 | CLASS | ID  | LENGTH | PAYLOAD | CK_A  | CK_B  |
 * | 0xB5  | 0x62  |  1B   | 1B  |   2B   |   NB    |  1B   |  1B   |
 * +-------+-------+-------+-----+--------+---------+-------+-------+
 * 
 * Total frame size  6 + N + 2 bytes (where N is payload length)
 * Checksum is calculated over the range: CLASS to PAYLOAD (inclusive)
 * 
 * @note: The u-blox module support two types of standards NMEA and UBX. NMEA is not implement in this library but could 
 *       be implemented in the future.
 * 
 * @version: 1.0
 * @sources:
 *   - u-blox MAX-M10 Interface Manual v5.10 (page of this manual will be referenced in comments)
 *     https://www.u-blox.com/en/product/max-m10-series#Documentation-&-resources
 *   - SparkFun u-blox GNSS Arduino Library v3
 *     https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3
 * 
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


// Class: CFG
#define UBX_CFG_VALSET_PKT_LEN 7 // (Version + Layers + Reserved + Bit Key Value) => 7 Bytes
#define UBX_CFG_VALSET 0x8A     // Set configuration item values

