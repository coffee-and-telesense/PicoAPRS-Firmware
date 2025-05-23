// aprsConfig.h

#ifndef APRSCONFIG_H
#define APRSCONFIG_H

// User configurable Mode definitions
#define DEBUG_MODE  //comment out to disable debug mode


// User configurable constant definitions
#define ANALOG_CHANNEL1_NAME "AbsTemp"  // max 7 characters
#define ANALOG_CHANNEL2_NAME "rtcTime"  // max 7 characters
#define ANALOG_CHANNEL3_NAME "Weight"   // max 6 characters
#define ANALOG_CHANNEL4_NAME "Charge"   // max 6 characters
#define ANALOG_CHANNEL5_NAME "aDist"    // max 5 characters

#define DIGITAL_CHANNEL1_NAME "Energy"  // max 6 characters
#define DIGITAL_CHANNEL2_NAME "HeatE"   // max 5 characters
#define DIGITAL_CHANNEL3_NAME "AMPS"    // max 4 characters
#define DIGITAL_CHANNEL4_NAME "Work"    // max 4 characters
#define DIGITAL_CHANNEL5_NAME "Oppo"    // max 4 characters
#define DIGITAL_CHANNEL6_NAME "LUX"     // max 3 characters
#define DIGITAL_CHANNEL7_NAME "BEC"     // max 3 characters
#define DIGITAL_CHANNEL8_NAME "RAD"     // max 3 characters

// User configurable constant definitions for analog channels
#define ANALOG_CHANNEL1_UNIT "Kelvin"   // max 7 characters
#define ANALOG_CHANNEL2_UNIT "Second"   // max 7 characters
#define ANALOG_CHANNEL3_UNIT "Gram"     // max 6 characters
#define ANALOG_CHANNEL4_UNIT "Ampere"   // max 6 characters
#define ANALOG_CHANNEL5_UNIT "Meter"    // max 5 characters

// User configurable constant definitions for digital channels
#define DIGITAL_CHANNEL1_UNIT "Volts"   // max 6 characters
#define DIGITAL_CHANNEL2_UNIT "Joule"   // max 5 characters
#define DIGITAL_CHANNEL3_UNIT "Amps"    // max 4 characters
#define DIGITAL_CHANNEL4_UNIT "Watt"    // max 4 characters
#define DIGITAL_CHANNEL5_UNIT "Ohms"    // max 4 characters
#define DIGITAL_CHANNEL6_UNIT "Lux"     // max 3 characters
#define DIGITAL_CHANNEL7_UNIT "Bec"     // max 3 characters (short for Becquerel)
#define DIGITAL_CHANNEL8_UNIT "Rad"     // max 3 characters (short for Radian)

#endif // APRSCONFIG_H
