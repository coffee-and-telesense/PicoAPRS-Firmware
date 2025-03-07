#ifndef APRS_H
#define APRS_H

#include <stdint.h>
#include <stddef.h>

// Constant definitions
#define TELEM_DATA_SIZE 34
#define MESSAGE_HEADER_SIZE 11
#define POSITION_HEADER_SIZE 27
#define SEQUENCE_NUMBER_SIZE 5
#define ANALOG_VALUE_SIZE 4
#define DIGITAL_VALUE_SIZE 8
#define ANALOG_CHANNEL1_SIZE 7
#define ANALOG_CHANNEL2_SIZE 7
#define ANALOG_CHANNEL3_SIZE 6
#define ANALOG_CHANNEL4_SIZE 6
#define ANALOG_CHANNEL5_SIZE 5
#define DIGITAL_CHANNEL1_SIZE 6
#define DIGITAL_CHANNEL2_SIZE 5
#define DIGITAL_CHANNEL3_SIZE 4
#define DIGITAL_CHANNEL4_SIZE 4
#define DIGITAL_CHANNEL5_SIZE 4
#define DIGITAL_CHANNEL6_SIZE 3
#define DIGITAL_CHANNEL7_SIZE 3
#define DIGITAL_CHANNEL8_SIZE 3
#define PARAM_IDENTIFIER_SIZE 5
#define UNIT_IDENTIFIER_SIZE 5
#define SIMPLEX_CAPIBILITY '/'
#define LATITUDE_SIZE 8
#define LONGITUDE_SIZE 9
#define TIME_SIZE 7


static const char SEQUENCE_NUMBER[SEQUENCE_NUMBER_SIZE + 1] = "#000,";


// Type Definitions
typedef struct {
    char typeIdentifier;
    uint8_t sequenceNumber[SEQUENCE_NUMBER_SIZE + 1];
    uint8_t analogValue1[ANALOG_VALUE_SIZE + 1];
    uint8_t analogValue2[ANALOG_VALUE_SIZE + 1];
    uint8_t analogValue3[ANALOG_VALUE_SIZE + 1];
    uint8_t analogValue4[ANALOG_VALUE_SIZE + 1];
    uint8_t analogValue5[ANALOG_VALUE_SIZE + 1];
    uint8_t digitalValue[DIGITAL_VALUE_SIZE + 1];
    char *comment;
    size_t commentLength;
    uint8_t *tData;
    size_t tDataSize;
} telemetryInfoFrame;

typedef struct {
    char typeIdentifier[PARAM_IDENTIFIER_SIZE + 1];
    char A1[ANALOG_CHANNEL1_SIZE + 1];
    char A2[ANALOG_CHANNEL2_SIZE + 1];
    char A3[ANALOG_CHANNEL3_SIZE + 1];
    char A4[ANALOG_CHANNEL4_SIZE + 1];
    char A5[ANALOG_CHANNEL5_SIZE + 1];
    char B1[DIGITAL_CHANNEL1_SIZE + 1];
    char B2[DIGITAL_CHANNEL2_SIZE + 1];
    char B3[DIGITAL_CHANNEL3_SIZE + 1];
    char B4[DIGITAL_CHANNEL4_SIZE + 1];
    char B5[DIGITAL_CHANNEL5_SIZE + 1];
    char B6[DIGITAL_CHANNEL6_SIZE + 1];
    char B7[DIGITAL_CHANNEL7_SIZE + 1];
    char B8[DIGITAL_CHANNEL8_SIZE + 1];
    uint8_t *paramFrame;
    size_t paramFrameSize;
} telemetryParamFrame;

typedef struct {
    char typeIdentifier[UNIT_IDENTIFIER_SIZE + 1];
    char AU1[ANALOG_CHANNEL1_SIZE + 1];
    char AU2[ANALOG_CHANNEL2_SIZE + 1];
    char AU3[ANALOG_CHANNEL3_SIZE + 1];
    char AU4[ANALOG_CHANNEL4_SIZE + 1];
    char AU5[ANALOG_CHANNEL5_SIZE + 1];
    char BU1[DIGITAL_CHANNEL1_SIZE + 1];
    char BU2[DIGITAL_CHANNEL2_SIZE + 1];
    char BU3[DIGITAL_CHANNEL3_SIZE + 1];
    char BU4[DIGITAL_CHANNEL4_SIZE + 1];
    char BU5[DIGITAL_CHANNEL5_SIZE + 1];
    char BU6[DIGITAL_CHANNEL6_SIZE + 1];
    char BU7[DIGITAL_CHANNEL7_SIZE + 1];
    char BU8[DIGITAL_CHANNEL8_SIZE + 1];
    uint8_t *unitFrame;
    size_t unitFrameSize;
} telemetryUnitFrame;

typedef struct {
    char typeIdentifier;
    char addressee[9];
    char endOfHeader;
    char *message;
    size_t messageSize;
    uint8_t *messageFrame;
    size_t messageFrameSize;
} messageFrame;

typedef struct {
    char capability;
    char time[TIME_SIZE + 1];
    char lat[LATITUDE_SIZE + 1];
    char tableID;
    char lon[LONGITUDE_SIZE + 1];
    char SymCode;
    char *comment;
    size_t commentSize;
    uint8_t *positionFrame;
    size_t positionFrameSize;
} positionFrame;

// Function prototypes
void concatTelemData(telemetryInfoFrame *tFrame);
void updateTelemData(telemetryInfoFrame *tFrame, uint8_t *analogValue1, uint8_t *analogValue2, uint8_t *analogValue3, uint8_t *analogValue4, uint8_t *analogValue5, uint8_t *digitalValue, char *comment);
telemetryInfoFrame *initTFrame(void);

void concatParamData(telemetryParamFrame *tParamFrame);
void updateTelemParamData(telemetryParamFrame *tParamFrame, char *A1, char *A2, char *A3, char *A4, char *A5, char *B1, char *B2, char *B3, char *B4, char *B5, char *B6, char *B7, char *B8);
telemetryParamFrame *initParamFrame(void);

void concatUnitData(telemetryUnitFrame *tUnitFrame);
void updateTelemUnitData(telemetryUnitFrame *tUnitFrame, char *AU1, char *AU2, char *AU3, char *AU4, char *AU5, char *BU1, char *BU2, char *BU3, char *BU4, char *BU5, char *BU6, char *BU7, char *BU8);
telemetryUnitFrame *initUnitFrame(void);

void concatMessageData(messageFrame *mFrame);
void updateMessageData(messageFrame *mFrame, char *addressee, char *message);
messageFrame *initMFrame(void);

void concatPositionData(positionFrame *pFrame);
void updatePositionData(positionFrame *pFrame, char *time, char *lat, char *lon, char *comment);
positionFrame *initPFrame(void);

void printTFrameStructMembers(telemetryInfoFrame *tFrame);
void printParamFrameStructMembers(telemetryParamFrame *tParamFrame);
void printUnitFrameStructMembers(telemetryUnitFrame *tUnitFrame);
void printMFrameStructMembers(messageFrame *mFrame);
void printPFrameStructMembers(positionFrame *pFrame);

#endif // APRS_H