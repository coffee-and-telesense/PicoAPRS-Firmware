/*******************************************************************************
 * @file: ubx_defs.h
 * @brief: UBX protocol definitions and constants for u-blox GNSS modules
 *
 * @note: Contains all protocol-specific constants including:
 *        - Frame structure constants
 *        - Message classes and IDs
 *        - Payload lengths
 *        - Communication settings
 * @also: Symbols which aren't curretly used in the code are commented out but 
 *        are left in the file for future reference or expansion. 
 ******************************************************************************/
#pragma once
#include "gps_types.h"
/*******************************************************************************
 * Frame Structure Constants
 ******************************************************************************/
#define UBX_SYNC_CHAR_1         0xB5    /**< First sync character of UBX frame */
#define UBX_SYNC_CHAR_2         0x62    /**< Second sync character of UBX frame */
#define UBX_HEADER_LENGTH       6       /**< Header length: 2 sync + 1 class + 1 id + 2 length */
#define UBX_CHECKSUM_LENGTH     2       /**< Checksum length in bytes */
#define UBX_MAX_PAYLOAD_LENGTH  (MAX_BUFFER_SIZE - UBX_HEADER_LENGTH - UBX_CHECKSUM_LENGTH)     /**< Maximum payload length (see Integration Manual p.24) */
#define UBX_MAX_PACKET_LENGTH   (UBX_HEADER_LENGTH + UBX_MAX_PAYLOAD_LENGTH + UBX_CHECKSUM_LENGTH)

/* Payload Length Constants in Bytes*/
#define UBX_NAV_STATUS_LEN  16    /**< Length of UBX-NAV-STATUS payload */
#define UBX_ACK_ACK_LEN     2     /**< Length of UBX-ACK-ACK payload */
#define UBX_CFG_VALSET_LEN  8     /**< Length of UBX-CFG-VALSET payload */
#define UBX_NAV_PVT_LEN     92    /**< Length of UBX-NAV-PVT payload */

/* Total Packet Lengths*/
#define UBX_ACK_PACKET_SIZE    (UBX_HEADER_LENGTH + UBX_ACK_ACK_LEN + UBX_CHECKSUM_LENGTH)

/*******************************************************************************
 * Message Classes
 ******************************************************************************/
#define UBX_CLASS_NAV 0x01  /**< Navigation Results: Position, Speed, Time, etc */
#define UBX_CLASS_RXM 0x02  /**< Receiver Manager Messages */
#define UBX_CLASS_INF 0x04  /**< Information Messages */
#define UBX_CLASS_ACK 0x05  /**< Ack/Nak Messages */
#define UBX_CLASS_CFG 0x06  /**< Configuration Input Messages */
#define UBX_CLASS_UPD 0x09  /**< Firmware Update Messages */
#define UBX_CLASS_MON 0x0A  /**< Monitoring Messages */
#define UBX_CLASS_AID 0x0B  /**< AssistNow Aiding Messages */
#define UBX_CLASS_TIM 0x0D  /**< Timing Messages */
#define UBX_CLASS_ESF 0x10  /**< External Sensor Fusion Messages */
#define UBX_CLASS_MGA 0x13  /**< Multiple GNSS Assistance Messages */
#define UBX_CLASS_LOG 0x21  /**< Logging Messages */
#define UBX_CLASS_SEC 0x27  /**< Security Feature Messages */
#define UBX_CLASS_HNR 0x28  /**< High Rate Navigation Results Messages */
#define UBX_CLASS_NMEA 0xF0 /**< NMEA Standard Messages */
#define UBX_CLASS_PUBX 0xF1 /**< u-blox Proprietary NMEA Messages */

/*******************************************************************************
 * Message IDs (by class)
 ******************************************************************************/
// Class: NAV
#define UBX_NAV_STATUS 0x03    /**< Receiver Navigation Status */
#define UBX_NAV_PVT    0x07    /**< Navigation Position Velocity Time Solution */

// Class: CFG
#define UBX_CFG_VALSET 0x8A    /**< Set configuration item values */
#define UBX_CFG_RST    0x04    /**< Reset Receiver / Clear Backup Data */

// Class: ACK
#define UBX_ACK_ACK    0x01    /**< Message Acknowledged */
#define UBX_ACK_NACK   0x00    /**< Message Not-Acknowledged */

/*******************************************************************************
 * Configuration Constants
 ******************************************************************************/
/* Config Keys (32-bit) - See Interface Description p.124 */
// Configuration layer definitions
#define UBX_CFG_LAYER_RAM    0x01
#define UBX_CFG_LAYER_BBR    0x10
#define UBX_CFG_LAYER_FLASH  0x20
