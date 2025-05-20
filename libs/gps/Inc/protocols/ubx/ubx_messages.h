/*******************************************************************************
 * @file: ubx_messages.h
 * @brief: Packet type definitions for u-blox MAX-M10S GNSS module
 *
 * This file contains the structure definitions for different UBX protocol message
 * payloads. Each structure maps to a specific message type and contains the
 * appropriate fields as defined in the u-blox interface manual.
 *
 * @important: The last declaration of this file is a union that allows for easy
 *            access to different types of payload structues using the same memory.
 *
 * @author: Reece Wayt
 * @date: January 25, 2025
 ******************************************************************************/

 #pragma once

 #include <stdint.h>


 /*******************************************************************************
  * UBX-NAV-STATUS Packet Type
  ******************************************************************************/
 /**
  * @brief UBX-NAV-STATUS payload structure
  * @note See pg. 105-107 of Interface Description
  */

 typedef struct
 {
   uint32_t iTOW;  // GPS time of week of the navigation epoch: ms
   uint8_t gpsFix; // GPSfix Type: 0x00 = no fix; 0x01 = dead reckoning only; 0x02 = 2D-fix; 0x03 = 3D-fix
                   // 0x04 = GPS + dead reckoning combined; 0x05 = Time only fix; 0x06..0xff = reserved
   union
   {
     uint8_t all;
     struct
     {
       uint8_t gpsFixOk : 1; // 1 = position and velocity valid and within DOP and ACC Masks.
       uint8_t diffSoln : 1; // 1 = differential corrections were applied
       uint8_t wknSet : 1;   // 1 = Week Number valid (see Time Validity section for details)
       uint8_t towSet : 1;   // 1 = Time of Week valid (see Time Validity section for details)
     } bits;
   } flags;
   union
   {
     uint8_t all;
     struct
     {
       uint8_t diffCorr : 1;      // 1 = differential corrections available
       uint8_t carrSolnValid : 1; // 1 = valid carrSoln
       uint8_t reserved : 4;
       uint8_t mapMatching : 2; // map matching status: 00: none
                                // 01: valid but not used, i.e. map matching data was received, but was too old
                                // 10: valid and used, map matching data was applied
                                // 11: valid and used, map matching data was applied.
     } bits;
   } fixStat;
   union
   {
     uint8_t all;
     struct
     {
       uint8_t psmState : 2; // power save mode state
                             // 0: ACQUISITION [or when psm disabled]
                             // 1: TRACKING
                             // 2: POWER OPTIMIZED TRACKING
                             // 3: INACTIVE
       uint8_t reserved1 : 1;
       uint8_t spoofDetState : 2; // Spoofing detection state
                                  // 0: Unknown or deactivated
                                  // 1: No spoofing indicated
                                  // 2: Spoofing indicated
                                  // 3: Multiple spoofing indications
       uint8_t reserved2 : 1;
       uint8_t carrSoln : 2; // Carrier phase range solution status:
                             // 0: no carrier phase range solution
                             // 1: carrier phase range solution with floating ambiguities
                             // 2: carrier phase range solution with fixed ambiguities
     } bits;
   } flags2;
   uint32_t ttff; // Time to first fix (millisecond time tag): ms
   uint32_t msss; // Milliseconds since Startup / Reset: ms
 } ubx_nav_status_s;

 /*******************************************************************************
  * UBX-NAV-PVT Packet Type
  *
  * @desciption: This packet type is used to store the data from the UBX-NAV-PVT
  *             message which provides position, velocity, and time information.
  ******************************************************************************/
 typedef struct
 {
   uint32_t iTOW; // GPS time of week of the navigation epoch: ms
   uint16_t year; // Year (UTC)
   uint8_t month; // Month, range 1..12 (UTC)
   uint8_t day;   // Day of month, range 1..31 (UTC)
   uint8_t hour;  // Hour of day, range 0..23 (UTC)
   uint8_t min;   // Minute of hour, range 0..59 (UTC)
   uint8_t sec;   // Seconds of minute, range 0..60 (UTC)
   union
   {
     uint8_t all;
     struct
     {
       uint8_t validDate : 1;     // 1 = valid UTC Date
       uint8_t validTime : 1;     // 1 = valid UTC time of day
       uint8_t fullyResolved : 1; // 1 = UTC time of day has been fully resolved (no seconds uncertainty).
       uint8_t validMag : 1;      // 1 = valid magnetic declination
     } bits;
   } valid;
   uint32_t tAcc;   // Time accuracy estimate (UTC): ns
   int32_t nano;    // Fraction of second, range -1e9 .. 1e9 (UTC): ns
   uint8_t fixType; // GNSSfix Type:
                    // 0: no fix
                    // 1: dead reckoning only
                    // 2: 2D-fix
                    // 3: 3D-fix
                    // 4: GNSS + dead reckoning combined
                    // 5: time only fix
   union
   {
     uint8_t all;
     struct
     {
       uint8_t gnssFixOK : 1; // 1 = valid fix (i.e within DOP & accuracy masks)
       uint8_t diffSoln : 1;  // 1 = differential corrections were applied
       uint8_t psmState : 3;
       uint8_t headVehValid : 1; // 1 = heading of vehicle is valid, only set if the receiver is in sensor fusion mode
       uint8_t carrSoln : 2;     // Carrier phase range solution status:
                                 // 0: no carrier phase range solution
                                 // 1: carrier phase range solution with floating ambiguities
                                 // 2: carrier phase range solution with fixed ambiguities
     } bits;
   } flags;
   union
   {
     uint8_t all;
     struct
     {
       uint8_t reserved : 5;
       uint8_t confirmedAvai : 1; // 1 = information about UTC Date and Time of Day validity confirmation is available
       uint8_t confirmedDate : 1; // 1 = UTC Date validity could be confirmed
       uint8_t confirmedTime : 1; // 1 = UTC Time of Day could be confirmed
     } bits;
   } flags2;
   uint8_t numSV;    // Number of satellites used in Nav Solution
   int32_t lon;      // Longitude: deg * 1e-7
   int32_t lat;      // Latitude: deg * 1e-7
   int32_t height;   // Height above ellipsoid: mm
   int32_t hMSL;     // Height above mean sea level: mm
   uint32_t hAcc;    // Horizontal accuracy estimate: mm
   uint32_t vAcc;    // Vertical accuracy estimate: mm
   int32_t velN;     // NED north velocity: mm/s
   int32_t velE;     // NED east velocity: mm/s
   int32_t velD;     // NED down velocity: mm/s
   int32_t gSpeed;   // Ground Speed (2-D): mm/s
   int32_t headMot;  // Heading of motion (2-D): deg * 1e-5
   uint32_t sAcc;    // Speed accuracy estimate: mm/s
   uint32_t headAcc; // Heading accuracy estimate (both motion and vehicle): deg * 1e-5
   uint16_t pDOP;    // Position DOP * 0.01
   union
   {
     uint16_t all;
     struct
     {
       uint16_t invalidLlh : 1;        // 1 = Invalid lon, lat, height and hMSL
       uint16_t lastCorrectionAge : 4; // Age of the most recently received differential correction:
                                      // 0: Not available
                                      // 1: Age between 0 and 1 second
                                      // 2: Age between 1 (inclusive) and 2 seconds
                                      // 3: Age between 2 (inclusive) and 5 seconds
                                      // 4: Age between 5 (inclusive) and 10 seconds
                                      // 5: Age between 10 (inclusive) and 15 seconds
                                      // 6: Age between 15 (inclusive) and 20 seconds
                                      // 7: Age between 20 (inclusive) and 30 seconds
                                      // 8: Age between 30 (inclusive) and 45 seconds
                                      // 9: Age between 45 (inclusive) and 60 seconds
                                      // 10: Age between 60 (inclusive) and 90 seconds
                                      // 11: Age between 90 (inclusive) and 120 seconds
                                      // >=12: Age greater or equal than 120 seconds
       uint16_t reserved : 8;
       uint16_t authTime : 1;     // Flag that indicates if the output time has been validated
                                  // against an external trusted time source
       uint16_t nmaFixStatus : 1; // Flag assigned to a fix that has been computed
                                  // mixing satellites with data authenticated through
                                  // Navigation Message Authentication (NMA) methods
                                  // and satellites using unauthenticated data.
     } bits;
   } flags3;
   uint8_t reserved0[4];
   int32_t headVeh; // Heading of vehicle (2-D): deg * 1e-5
   int16_t magDec;  // Magnetic declination: deg * 1e-2
   uint16_t magAcc; // Magnetic declination accuracy: deg * 1e-2
 } ubx_nav_pvt_s;


 /*******************************************************************************
  * @brief UBX-ACK-ACK & UBX-ACK_NACK payload structure
  * @note: The payload structure is the same for both ACK and NACK messages
  *          The only difference is that the ID will be 0x01 for ACK and 0x00 for NACK
  *          See pg. 49 of Interface Description
  ********************************************************************************/
 typedef struct
 {
   uint8_t clsID; // This is an echo message of the Class and ID of the original config message
   uint8_t msgID;
 } ubx_ack_ack_s;
