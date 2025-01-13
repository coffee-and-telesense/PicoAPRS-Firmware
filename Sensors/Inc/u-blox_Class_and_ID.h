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
 * Total frame size = 6 + N + 2 bytes (where N is payload length)
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

//@TODO: Add more MCU_TARGETs
// For example MCU_TARGET == F091RC
#if TARGET_MCU == STM32L432KC
  #include "stm32l4xx_hal.h"
#endif

const uint8_t UBX_SYNCH_1 = 0xB5;
const uint8_t UBX_SYNCH_2 = 0x62;

/*******************************************************************************
 * Class Message Types
********************************************************************************/
//UBX Message Class. See Chapter 3
const uint8_t UBX_CLASS_NAV = 0x01;  // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used
const uint8_t UBX_CLASS_RXM = 0x02;  // Receiver Manager Messages: Satellite Status, RTC Status
const uint8_t UBX_CLASS_INF = 0x04;  // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
const uint8_t UBX_CLASS_ACK = 0x05;  // Ack/Nak Messages: Acknowledge or Reject messages to UBX-CFG input messages
const uint8_t UBX_CLASS_CFG = 0x06;  // Configuration Input Messages: Configure the receiver.
const uint8_t UBX_CLASS_UPD = 0x09;  // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
const uint8_t UBX_CLASS_MON = 0x0A;  // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
const uint8_t UBX_CLASS_AID = 0x0B;  //(NEO-M8P ONLY!!!) AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
const uint8_t UBX_CLASS_TIM = 0x0D;  // Timing Messages: Time Pulse Output, Time Mark Results
const uint8_t UBX_CLASS_ESF = 0x10;  //(NEO-M8P ONLY!!!) External Sensor Fusion Messages: External Sensor Measurements and Status Information
const uint8_t UBX_CLASS_MGA = 0x13;  // Multiple GNSS Assistance Messages: Assistance data for various GNSS
const uint8_t UBX_CLASS_LOG = 0x21;  // Logging Messages: Log creation, deletion, info and retrieval
const uint8_t UBX_CLASS_SEC = 0x27;  // Security Feature Messages
const uint8_t UBX_CLASS_HNR = 0x28;  //(NEO-M8P ONLY!!!) High Rate Navigation Results Messages: High rate time, position speed, heading
//@TODO: NMEA messages are not implemented in this library
const uint8_t UBX_CLASS_NMEA = 0xF0; // NMEA Strings: standard NMEA strings
const uint8_t UBX_CLASS_PUBX = 0xF1; // Proprietary NMEA-format messages defined by u-blox


/*******************************************************************************
 * Message IDs
********************************************************************************/

// Class: CFG
// The following are used for configuration.
const uint8_t UBX_CFG_ANT = 0x13;       // Antenna Control Settings. Used to configure the antenna control settings
const uint8_t UBX_CFG_BATCH = 0x93;     // Get/set data batching configuration.
const uint8_t UBX_CFG_CFG = 0x09;       // Clear, Save, and Load Configurations. Used to save current configuration
const uint8_t UBX_CFG_DAT = 0x06;       // Set User-defined Datum or The currently defined Datum
const uint8_t UBX_CFG_DGNSS = 0x70;     // DGNSS configuration
const uint8_t UBX_CFG_ESFALG = 0x56;    // ESF alignment
const uint8_t UBX_CFG_ESFA = 0x4C;      // ESF accelerometer
const uint8_t UBX_CFG_ESFG = 0x4D;      // ESF gyro
const uint8_t UBX_CFG_GEOFENCE = 0x69;  // Geofencing configuration. Used to configure a geofence
const uint8_t UBX_CFG_GNSS = 0x3E;      // GNSS system configuration
const uint8_t UBX_CFG_HNR = 0x5C;       // High Navigation Rate
const uint8_t UBX_CFG_INF = 0x02;       // Depending on packet length, either: poll configuration for one protocol, or information message configuration
const uint8_t UBX_CFG_ITFM = 0x39;      // Jamming/Interference Monitor configuration
const uint8_t UBX_CFG_LOGFILTER = 0x47; // Data Logger Configuration
const uint8_t UBX_CFG_MSG = 0x01;       // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
const uint8_t UBX_CFG_NAV5 = 0x24;      // Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
const uint8_t UBX_CFG_NAVX5 = 0x23;     // Navigation Engine Expert Settings
const uint8_t UBX_CFG_NMEA = 0x17;      // Extended NMEA protocol configuration V1
const uint8_t UBX_CFG_ODO = 0x1E;       // Odometer, Low-speed COG Engine Settings
const uint8_t UBX_CFG_PM2 = 0x3B;       // Extended power management configuration
const uint8_t UBX_CFG_PMS = 0x86;       // Power mode setup
const uint8_t UBX_CFG_PRT = 0x00;       // Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
const uint8_t UBX_CFG_PWR = 0x57;       // Put receiver in a defined power state
const uint8_t UBX_CFG_RATE = 0x08;      // Navigation/Measurement Rate Settings. Used to set port baud rates.
const uint8_t UBX_CFG_RINV = 0x34;      // Contents of Remote Inventory
const uint8_t UBX_CFG_RST = 0x04;       // Reset Receiver / Clear Backup Data Structures. Used to reset device.
const uint8_t UBX_CFG_RXM = 0x11;       // RXM configuration
const uint8_t UBX_CFG_SBAS = 0x16;      // SBAS configuration
const uint8_t UBX_CFG_TMODE3 = 0x71;    // Time Mode Settings 3. Used to enable Survey In Mode
const uint8_t UBX_CFG_TP5 = 0x31;       // Time Pulse Parameters
const uint8_t UBX_CFG_USB = 0x1B;       // USB Configuration
const uint8_t UBX_CFG_VALDEL = 0x8C;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
const uint8_t UBX_CFG_VALGET = 0x8B;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
const uint8_t UBX_CFG_VALSET = 0x8A;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.


// Class: PUBX
// The following are used to enable PUBX messages with configureMessage
// See the M8 receiver description & protocol specification for more details
const uint8_t UBX_PUBX_CONFIG = 0x41;   // Set protocols and baud rate
const uint8_t UBX_PUBX_POSITION = 0x00; // Lat/Long position data
const uint8_t UBX_PUBX_RATE = 0x40;     // Set/get NMEA message output rate
const uint8_t UBX_PUBX_SVSTATUS = 0x03; // Satellite status
const uint8_t UBX_PUBX_TIME = 0x04;     // Time of day and clock information

// Class: HNR
// The following are used to configure the HNR message rates
const uint8_t UBX_HNR_ATT = 0x01; // HNR Attitude
const uint8_t UBX_HNR_INS = 0x02; // HNR Vehicle Dynamics
const uint8_t UBX_HNR_PVT = 0x00; // HNR PVT

// Class: INF
// The following are used to configure INF UBX messages (information messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_INF_CLASS = 0x04;   // All INF messages have 0x04 as the class
const uint8_t UBX_INF_DEBUG = 0x04;   // ASCII output with debug contents
const uint8_t UBX_INF_ERROR = 0x00;   // ASCII output with error contents
const uint8_t UBX_INF_NOTICE = 0x02;  // ASCII output with informational contents
const uint8_t UBX_INF_TEST = 0x03;    // ASCII output with test contents
const uint8_t UBX_INF_WARNING = 0x01; // ASCII output with warning contents

// Class: LOG
// The following are used to configure LOG UBX messages (loggings messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_LOG_CREATE = 0x07;           // Create Log File
const uint8_t UBX_LOG_ERASE = 0x03;            // Erase Logged Data
const uint8_t UBX_LOG_FINDTIME = 0x0E;         // Find index of a log entry based on a given time, or response to FINDTIME requested
const uint8_t UBX_LOG_INFO = 0x08;             // Poll for log information, or Log information
const uint8_t UBX_LOG_RETRIEVEPOSEXTRA = 0x0F; // Odometer log entry
const uint8_t UBX_LOG_RETRIEVEPOS = 0x0B;      // Position fix log entry
const uint8_t UBX_LOG_RETRIEVESTRING = 0x0D;   // Byte string log entry
const uint8_t UBX_LOG_RETRIEVE = 0x09;         // Request log data
const uint8_t UBX_LOG_STRING = 0x04;           // Store arbitrary string on on-board flash

// Class: MGA
// The following are used to configure MGA UBX messages (Multiple GNSS Assistance Messages).  Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 34)
const uint8_t UBX_MGA_ACK_DATA0 = 0x60;      // Multiple GNSS Acknowledge message
const uint8_t UBX_MGA_ANO = 0x20;            // Multiple GNSS AssistNow Offline assistance - NOT SUPPORTED BY THE ZED-F9P! "The ZED-F9P supports AssistNow Online only."
const uint8_t UBX_MGA_BDS_EPH = 0x03;        // BDS Ephemeris Assistance
const uint8_t UBX_MGA_BDS_ALM = 0x03;        // BDS Almanac Assistance
const uint8_t UBX_MGA_BDS_HEALTH = 0x03;     // BDS Health Assistance
const uint8_t UBX_MGA_BDS_UTC = 0x03;        // BDS UTC Assistance
const uint8_t UBX_MGA_BDS_IONO = 0x03;       // BDS Ionospheric Assistance
const uint8_t UBX_MGA_DBD = 0x80;            // Either: Poll the Navigation Database, or Navigation Database Dump Entry
const uint8_t UBX_MGA_GAL_EPH = 0x02;        // Galileo Ephemeris Assistance
const uint8_t UBX_MGA_GAL_ALM = 0x02;        // Galileo Almanac Assitance
const uint8_t UBX_MGA_GAL_TIMOFFSET = 0x02;  // Galileo GPS time offset assistance
const uint8_t UBX_MGA_GAL_UTC = 0x02;        // Galileo UTC Assistance
const uint8_t UBX_MGA_GLO_EPH = 0x06;        // GLONASS Ephemeris Assistance
const uint8_t UBX_MGA_GLO_ALM = 0x06;        // GLONASS Almanac Assistance
const uint8_t UBX_MGA_GLO_TIMEOFFSET = 0x06; // GLONASS Auxiliary Time Offset Assistance
const uint8_t UBX_MGA_GPS_EPH = 0x00;        // GPS Ephemeris Assistance
const uint8_t UBX_MGA_GPS_ALM = 0x00;        // GPS Almanac Assistance
const uint8_t UBX_MGA_GPS_HEALTH = 0x00;     // GPS Health Assistance
const uint8_t UBX_MGA_GPS_UTC = 0x00;        // GPS UTC Assistance
const uint8_t UBX_MGA_GPS_IONO = 0x00;       // GPS Ionosphere Assistance
const uint8_t UBX_MGA_INI_POS_XYZ = 0x40;    // Initial Position Assistance
const uint8_t UBX_MGA_INI_POS_LLH = 0x40;    // Initial Position Assitance
const uint8_t UBX_MGA_INI_TIME_UTC = 0x40;   // Initial Time Assistance
const uint8_t UBX_MGA_INI_TIME_GNSS = 0x40;  // Initial Time Assistance
const uint8_t UBX_MGA_INI_CLKD = 0x40;       // Initial Clock Drift Assitance
const uint8_t UBX_MGA_INI_FREQ = 0x40;       // Initial Frequency Assistance
const uint8_t UBX_MGA_INI_EOP = 0x40;        // Earth Orientation Parameters Assistance
const uint8_t UBX_MGA_QZSS_EPH = 0x05;       // QZSS Ephemeris Assistance
const uint8_t UBX_MGA_QZSS_ALM = 0x05;       // QZSS Almanac Assistance
const uint8_t UBX_MGA_QZAA_HEALTH = 0x05;    // QZSS Health Assistance

// Class: MON
// The following are used to configure the MON UBX messages (monitoring messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35)
const uint8_t UBX_MON_COMMS = 0x36; // Comm port information
const uint8_t UBX_MON_GNSS = 0x28;  // Information message major GNSS selection
const uint8_t UBX_MON_HW2 = 0x0B;   // Extended Hardware Status
const uint8_t UBX_MON_HW3 = 0x37;   // HW I/O pin information
const uint8_t UBX_MON_HW = 0x09;    // Hardware Status
const uint8_t UBX_MON_IO = 0x02;    // I/O Subsystem Status
const uint8_t UBX_MON_MSGPP = 0x06; // Message Parse and Process Status
const uint8_t UBX_MON_PATCH = 0x27; // Output information about installed patches
const uint8_t UBX_MON_PMP = 0x35;   // PMP monitoring data
const uint8_t UBX_MON_PT2 = 0x2B;   // Multi-GNSS production test monitor
const uint8_t UBX_MON_RF = 0x38;    // RF information
const uint8_t UBX_MON_RXBUF = 0x07; // Receiver Buffer Status
const uint8_t UBX_MON_RXR = 0x21;   // Receiver Status Information
const uint8_t UBX_MON_SPAN = 0x31;  // Signal characteristics
const uint8_t UBX_MON_SYS = 0x39;   // Current system performance information
const uint8_t UBX_MON_TEMP = 0x0E;  // Temperature value and state
const uint8_t UBX_MON_TXBUF = 0x08; // Transmitter Buffer Status. Used for query tx buffer size/state.
const uint8_t UBX_MON_VER = 0x04;   // Receiver/Software Version. Used for obtaining Protocol Version.

// Class: NAV
// The following are used to configure the NAV UBX messages (navigation results messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 35-36)
const uint8_t UBX_NAV_ATT = 0x05;       // Vehicle "Attitude" Solution
const uint8_t UBX_NAV_CLOCK = 0x22;     // Clock Solution
const uint8_t UBX_NAV_COV = 0x36;       // Covariance matrices
const uint8_t UBX_NAV_DOP = 0x04;       // Dilution of precision
const uint8_t UBX_NAV_EELL = 0x3D;      // Position error ellipse parameters
const uint8_t UBX_NAV_EOE = 0x61;       // End of Epoch
const uint8_t UBX_NAV_GEOFENCE = 0x39;  // Geofencing status. Used to poll the geofence status
const uint8_t UBX_NAV_HPPOSECEF = 0x13; // High Precision Position Solution in ECEF. Used to find our positional accuracy (high precision).
const uint8_t UBX_NAV_HPPOSLLH = 0x14;  // High Precision Geodetic Position Solution. Used for obtaining lat/long/alt in high precision
const uint8_t UBX_NAV_ODO = 0x09;       // Odometer Solution
const uint8_t UBX_NAV_ORB = 0x34;       // GNSS Orbit Database Info
const uint8_t UBX_NAV_PL = 0x62;        // Protection Level Information
const uint8_t UBX_NAV_POSECEF = 0x01;   // Position Solution in ECEF
const uint8_t UBX_NAV_POSLLH = 0x02;    // Geodetic Position Solution
const uint8_t UBX_NAV_PVT = 0x07;       // All the things! Position, velocity, time, PDOP, height, h/v accuracies, number of satellites. Navigation Position Velocity Time Solution.
const uint8_t UBX_NAV_PVAT = 0x17;      // Navigation position velocity attitude time solution (ZED-F9R only)
const uint8_t UBX_NAV_RELPOSNED = 0x3C; // Relative Positioning Information in NED frame
const uint8_t UBX_NAV_RESETODO = 0x10;  // Reset odometer
const uint8_t UBX_NAV_SAT = 0x35;       // Satellite Information
const uint8_t UBX_NAV_SBAS = 0x32;      // SBAS subsystem
const uint8_t UBX_NAV_SLAS = 0x42;      // QZSS L1S SLAS status data
const uint8_t UBX_NAV_SIG = 0x43;       // Signal Information
const uint8_t UBX_NAV_STATUS = 0x03;    // Receiver Navigation Status
const uint8_t UBX_NAV_SVIN = 0x3B;      // Survey-in data. Used for checking Survey In status
const uint8_t UBX_NAV_TIMEBDS = 0x24;   // BDS Time Solution
const uint8_t UBX_NAV_TIMEGAL = 0x25;   // Galileo Time Solution
const uint8_t UBX_NAV_TIMEGLO = 0x23;   // GLO Time Solution
const uint8_t UBX_NAV_TIMEGPS = 0x20;   // GPS Time Solution
const uint8_t UBX_NAV_TIMELS = 0x26;    // Leap second event information
const uint8_t UBX_NAV_TIMENAVIC = 0x63; // NavIC time solution
const uint8_t UBX_NAV_TIMEUTC = 0x21;   // UTC Time Solution
const uint8_t UBX_NAV_VELECEF = 0x11;   // Velocity Solution in ECEF
const uint8_t UBX_NAV_VELNED = 0x12;    // Velocity Solution in NED
const uint8_t UBX_NAV_AOPSTATUS = 0x60; // AssistNow Autonomous status

// Class: RXM
// The following are used to configure the RXM UBX messages (receiver manager messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_RXM_COR = 0x34;       // Differential correction input status
const uint8_t UBX_RXM_MEASX = 0x14;     // Satellite Measurements for RRLP
const uint8_t UBX_RXM_PMP = 0x72;       // PMP raw data (NEO-D9S) (two different versions) (packet size for version 0x01 is variable)
const uint8_t UBX_RXM_QZSSL6 = 0x73;    // QZSSL6 data (NEO-D9C)
const uint8_t UBX_RXM_PMREQ = 0x41;     // Requests a Power Management task (two different packet sizes)
const uint8_t UBX_RXM_RAWX = 0x15;      // Multi-GNSS Raw Measurement Data
const uint8_t UBX_RXM_RLM = 0x59;       // Galileo SAR Short-RLM report (two different packet sizes)
const uint8_t UBX_RXM_RTCM = 0x32;      // RTCM input status
const uint8_t UBX_RXM_SFRBX = 0x13;     // Broadcast Navigation Data Subframe
const uint8_t UBX_RXM_SPARTN = 0x33;    // SPARTN input status
const uint8_t UBX_RXM_SPARTNKEY = 0x36; // Poll/transfer dynamic SPARTN keys

// Class: SEC
// The following are used to configure the SEC UBX messages (security feature messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_SEC_UNIQID = 0x03; // Unique chip ID

// Class: TIM
// The following are used to configure the TIM UBX messages (timing messages). Descriptions from UBX messages overview (ZED_F9P Interface Description Document page 36)
const uint8_t UBX_TIM_TM2 = 0x03;  // Time mark data
const uint8_t UBX_TIM_TP = 0x01;   // Time Pulse Timedata
const uint8_t UBX_TIM_VRFY = 0x06; // Sourced Time Verification

// Class: UPD
// The following are used to configure the UPD UBX messages (firmware update messages). Descriptions from UBX messages overview (ZED-F9P Interface Description Document page 36)
const uint8_t UBX_UPD_SOS = 0x14; // Poll Backup Fil Restore Status, Create Backup File in Flash, Clear Backup File in Flash, Backup File Creation Acknowledge, System Restored from Backup

// The following are used to enable RTCM messages
const uint8_t UBX_RTCM_MSB = 0xF5;    // All RTCM enable commands have 0xF5 as MSB
const uint8_t UBX_RTCM_1005 = 0x05;   // Stationary RTK reference ARP
const uint8_t UBX_RTCM_1074 = 0x4A;   // GPS MSM4
const uint8_t UBX_RTCM_1077 = 0x4D;   // GPS MSM7
const uint8_t UBX_RTCM_1084 = 0x54;   // GLONASS MSM4
const uint8_t UBX_RTCM_1087 = 0x57;   // GLONASS MSM7
const uint8_t UBX_RTCM_1094 = 0x5E;   // Galileo MSM4
const uint8_t UBX_RTCM_1097 = 0x61;   // Galileo MSM7
const uint8_t UBX_RTCM_1124 = 0x7C;   // BeiDou MSM4
const uint8_t UBX_RTCM_1127 = 0x7F;   // BeiDou MSM7
const uint8_t UBX_RTCM_1230 = 0xE6;   // GLONASS code-phase biases, set to once every 10 seconds
const uint8_t UBX_RTCM_4072_0 = 0xFE; // Reference station PVT (ublox proprietary RTCM message)
const uint8_t UBX_RTCM_4072_1 = 0xFD; // Additional reference station information (ublox proprietary RTCM message)

// Class: ACK
const uint8_t UBX_ACK_NACK = 0x00;
const uint8_t UBX_ACK_ACK = 0x01;
const uint8_t UBX_ACK_NONE = 0x02; // Not a real value

// Class: ESF
//  The following constants are used to get External Sensor Measurements and Status
//  Information.
const uint8_t UBX_ESF_MEAS = 0x02;
const uint8_t UBX_ESF_RAW = 0x03;
const uint8_t UBX_ESF_STATUS = 0x10;
const uint8_t UBX_ESF_RESETALG = 0x13;
const uint8_t UBX_ESF_ALG = 0x14;
const uint8_t UBX_ESF_INS = 0x15; // 36 bytes

// CFG-TMODE: Time mode configuration
const uint8_t SVIN_MODE_DISABLE = 0x00;
const uint8_t SVIN_MODE_ENABLE = 0x01; // Survey-In
const uint8_t SVIN_MODE_FIXED = 0x02;

// The following consts are used to configure the various ports and streams for those ports. See -CFG-PRT.
const uint8_t COM_PORT_I2C = 0;
const uint8_t COM_PORT_UART1 = 1;
const uint8_t COM_PORT_UART2 = 2;
const uint8_t COM_PORT_USB = 3;
const uint8_t COM_PORT_SPI = 4;

// Port IDs used by MON-COMMS
// UBX-18010802 - R15 also documents 0x0101 and 0x0200 - both are "Reserved"
const uint16_t COM_PORT_ID_I2C = 0x0000; // Port IDs used by MON-COMMS
const uint16_t COM_PORT_ID_UART1 = 0x0100;
const uint16_t COM_PORT_ID_UART2 = 0x0201;
const uint16_t COM_PORT_ID_USB = 0x0300;
const uint16_t COM_PORT_ID_SPI = 0x0400;

const uint8_t COM_TYPE_UBX = (1 << 0);
const uint8_t COM_TYPE_NMEA = (1 << 1);
const uint8_t COM_TYPE_RTCM3 = (1 << 5);
const uint8_t COM_TYPE_SPARTN = (1 << 6);

// Odometer configuration - flags
const uint8_t UBX_CFG_ODO_USE_ODO = (1 << 0);
const uint8_t UBX_CFG_ODO_USE_COG = (1 << 1);
const uint8_t UBX_CFG_ODO_OUT_LP_VEL = (1 << 2);
const uint8_t UBX_CFG_ODO_OUT_LP_COG = (1 << 3);

// Odometer configuration - odoCfg
enum odoCfg_e
{
  UBX_CFG_ODO_RUN = 0,
  UBX_CFG_ODO_CYCLE,
  UBX_CFG_ODO_SWIM,
  UBX_CFG_ODO_CAR,
  UBX_CFG_ODO_CUSTOM,
};

// Configuration Sub-Section mask definitions for saveConfigSelective (UBX-CFG-CFG)
const uint32_t VAL_CFG_SUBSEC_IOPORT = 0x00000001;   // ioPort - communications port settings (causes IO system reset!)
const uint32_t VAL_CFG_SUBSEC_MSGCONF = 0x00000002;  // msgConf - message configuration
const uint32_t VAL_CFG_SUBSEC_INFMSG = 0x00000004;   // infMsg - INF message configuration
const uint32_t VAL_CFG_SUBSEC_NAVCONF = 0x00000008;  // navConf - navigation configuration
const uint32_t VAL_CFG_SUBSEC_RXMCONF = 0x00000010;  // rxmConf - receiver manager configuration
const uint32_t VAL_CFG_SUBSEC_SENCONF = 0x00000100;  // senConf - sensor interface configuration (requires protocol 19+)
const uint32_t VAL_CFG_SUBSEC_RINVCONF = 0x00000200; // rinvConf - remove inventory configuration
const uint32_t VAL_CFG_SUBSEC_ANTCONF = 0x00000400;  // antConf - antenna configuration
const uint32_t VAL_CFG_SUBSEC_LOGCONF = 0x00000800;  // logConf - logging configuration
const uint32_t VAL_CFG_SUBSEC_FTSCONF = 0x00001000;  // ftsConf - FTS configuration (FTS products only)

// Bitfield wakeupSources for UBX_RXM_PMREQ
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX = 0x00000008;  // uartrx
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT0 = 0x00000020; // extint0
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_EXTINT1 = 0x00000040; // extint1
const uint32_t VAL_RXM_PMREQ_WAKEUPSOURCE_SPICS = 0x00000080;   // spics

