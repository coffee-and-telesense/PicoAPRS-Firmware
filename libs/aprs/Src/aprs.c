
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "aprs.h"
#include "aprsConfig.h"
#include "ax25.h"


telemetryInfoFrame *initTFrame(void)
{

    telemetryInfoFrame *tFrame = (telemetryInfoFrame *)malloc(sizeof(telemetryInfoFrame));
    if (tFrame == NULL)
    {
        return NULL;
    } else {
        printf("Telemetry Frame Initialized!\n");
    }


    tFrame->typeIdentifier = 'T';

    memset(tFrame->sequenceNumber, 0, sizeof(tFrame->sequenceNumber));
    memset(tFrame->analogValue1, 0, sizeof(tFrame->analogValue1));
    memset(tFrame->analogValue2, 0, sizeof(tFrame->analogValue2));
    memset(tFrame->analogValue3, 0, sizeof(tFrame->analogValue3));
    memset(tFrame->analogValue4, 0, sizeof(tFrame->analogValue4));
    memset(tFrame->analogValue5, 0, sizeof(tFrame->analogValue5));
    memset(tFrame->digitalValue, 0, sizeof(tFrame->digitalValue));

    tFrame->comment = NULL;
    tFrame->commentLength = 0;
    tFrame->tData = NULL;
    tFrame->tDataSize = 0;

    return tFrame;

}


void updateTelemData(telemetryInfoFrame *tFrame, uint8_t *analogValue1, uint8_t *analogValue2, uint8_t *analogValue3, uint8_t *analogValue4, uint8_t *analogValue5, uint8_t *digitalValue, char *comment)
{

memcpy(tFrame->sequenceNumber, SEQUENCE_NUMBER, sizeof(tFrame->sequenceNumber));
    memcpy(tFrame->analogValue1, analogValue1, sizeof(tFrame->analogValue1));
    memcpy(tFrame->analogValue2, analogValue2, sizeof(tFrame->analogValue2));
    memcpy(tFrame->analogValue3, analogValue3, sizeof(tFrame->analogValue3));
    memcpy(tFrame->analogValue4, analogValue4, sizeof(tFrame->analogValue4));
    memcpy(tFrame->analogValue5, analogValue5, sizeof(tFrame->analogValue5));
    memcpy(tFrame->digitalValue, digitalValue, sizeof(tFrame->digitalValue));

    tFrame->comment = comment;
    tFrame->commentLength = strlen(comment);
    tFrame->tData = NULL;
    tFrame->tDataSize = 0;

    printTFrameStructMembers(tFrame);

}

void concatTelemData(telemetryInfoFrame *tFrame)
{

    uint8_t *telemData = (uint8_t *)malloc((sizeof(uint8_t) * TELEM_DATA_SIZE) + (sizeof(uint8_t) * tFrame->commentLength + 1));

    tFrame->tDataSize = TELEM_DATA_SIZE + tFrame->commentLength;

    telemData[0] = tFrame->typeIdentifier;

    snprintf((char *)telemData, tFrame->tDataSize + 1,
             "%c%s%s%s%s%s%s%s%s",
             tFrame->typeIdentifier,
             tFrame->sequenceNumber,
             tFrame->analogValue1,
             tFrame->analogValue2,
             tFrame->analogValue3,
             tFrame->analogValue4,
             tFrame->analogValue5,
             tFrame->digitalValue,
             tFrame->comment);

    tFrame->tData = (uint8_t *)telemData;

    printf("Concatenated Telemetry Data: %.*s\n", (int)tFrame->tDataSize+1, tFrame->tData);
    printf("Telemetry Data Size: %ld\n", tFrame->tDataSize);
    printf("--------------------------------------------\n");
}

telemetryParamFrame *initParamFrame(void)
{
    telemetryParamFrame *tParamFrame = (telemetryParamFrame *)malloc(sizeof(telemetryParamFrame));
    if (tParamFrame == NULL)
    {
        return NULL;
    } else {
        printf("Parameter Frame Initialized!\n");
    }

    strcpy(tParamFrame->typeIdentifier, "PARM.");
    tParamFrame->paramFrameSize = PARAM_IDENTIFIER_SIZE;

    memset(tParamFrame->A1, 0, sizeof(tParamFrame->A1));
    memset(tParamFrame->A2, 0, sizeof(tParamFrame->A2));
    memset(tParamFrame->A3, 0, sizeof(tParamFrame->A3));
    memset(tParamFrame->A4, 0, sizeof(tParamFrame->A4));
    memset(tParamFrame->A5, 0, sizeof(tParamFrame->A5));
    memset(tParamFrame->B1, 0, sizeof(tParamFrame->B1));
    memset(tParamFrame->B2, 0, sizeof(tParamFrame->B2));
    memset(tParamFrame->B3, 0, sizeof(tParamFrame->B3));
    memset(tParamFrame->B4, 0, sizeof(tParamFrame->B4));
    memset(tParamFrame->B5, 0, sizeof(tParamFrame->B5));
    memset(tParamFrame->B6, 0, sizeof(tParamFrame->B6));
    memset(tParamFrame->B7, 0, sizeof(tParamFrame->B7));
    memset(tParamFrame->B8, 0, sizeof(tParamFrame->B8));
    
    tParamFrame->paramFrame = NULL;

    return tParamFrame;
}

void updateTelemParamData(telemetryParamFrame *tParamFrame, char *A1, char *A2, char *A3, char *A4, char *A5, char *B1, char *B2, char *B3, char *B4, char *B5, char *B6, char *B7, char *B8)
{
    char *analog[] = {tParamFrame->A1, tParamFrame->A2, tParamFrame->A3, tParamFrame->A4, tParamFrame->A5};
    char *digital[] = {tParamFrame->B1, tParamFrame->B2, tParamFrame->B3, tParamFrame->B4, tParamFrame->B5, tParamFrame->B6, tParamFrame->B7, tParamFrame->B8};
    char *inputAnalog[] = {A1, A2, A3, A4, A5};
    char *inputDigital[] = {B1, B2, B3, B4, B5, B6, B7, B8};

    for (int i = 0; i < 5; i++)
    {
        strcpy(analog[i], inputAnalog[i]);
        tParamFrame->paramFrameSize += strlen(inputAnalog[i]);
    }

    for (int i = 0; i < 8; i++)
    {
        strcpy(digital[i], inputDigital[i]);
        tParamFrame->paramFrameSize += strlen(inputDigital[i]);
    }

    printParamFrameStructMembers(tParamFrame);
}

void concatParamData(telemetryParamFrame *tParamFrame)
{
    uint8_t *paramData = (uint8_t *)malloc((sizeof(uint8_t) * tParamFrame->paramFrameSize + 1));

    // int offset = 0;

    snprintf((char *)paramData, tParamFrame->paramFrameSize + 1,
             "%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
             tParamFrame->typeIdentifier,
             tParamFrame->A1,
             tParamFrame->A2,
             tParamFrame->A3,
             tParamFrame->A4,
             tParamFrame->A5,
             tParamFrame->B1,
             tParamFrame->B2,
             tParamFrame->B3,
             tParamFrame->B4,
             tParamFrame->B5,
             tParamFrame->B6,
             tParamFrame->B7,
             tParamFrame->B8);

    tParamFrame->paramFrame = (uint8_t *)paramData;
    printf("Concatenated Param Data: %.*s\n", (int)tParamFrame->paramFrameSize, tParamFrame->paramFrame);
    printf("--------------------------------------------\n");
    return;
}

telemetryUnitFrame *initUnitFrame(void)
{

    telemetryUnitFrame *tUnitFrame = (telemetryUnitFrame *)malloc(sizeof(telemetryUnitFrame));

    if (tUnitFrame == NULL)
    {
        return NULL;
    }else {
        printf("Unit Frame Initialized!\n");
    }


    strcpy(tUnitFrame->typeIdentifier, "UNIT.");
    tUnitFrame->unitFrameSize = UNIT_IDENTIFIER_SIZE;
    memset(tUnitFrame->AU1, 0, sizeof(tUnitFrame->AU1));
    memset(tUnitFrame->AU2, 0, sizeof(tUnitFrame->AU2));
    memset(tUnitFrame->AU3, 0, sizeof(tUnitFrame->AU3));
    memset(tUnitFrame->AU4, 0, sizeof(tUnitFrame->AU4));
    memset(tUnitFrame->AU5, 0, sizeof(tUnitFrame->AU5));
    memset(tUnitFrame->BU1, 0, sizeof(tUnitFrame->BU1));
    memset(tUnitFrame->BU2, 0, sizeof(tUnitFrame->BU2));
    memset(tUnitFrame->BU3, 0, sizeof(tUnitFrame->BU3));
    memset(tUnitFrame->BU4, 0, sizeof(tUnitFrame->BU4));
    memset(tUnitFrame->BU5, 0, sizeof(tUnitFrame->BU5));
    memset(tUnitFrame->BU6, 0, sizeof(tUnitFrame->BU6));
    memset(tUnitFrame->BU7, 0, sizeof(tUnitFrame->BU7));
    memset(tUnitFrame->BU8, 0, sizeof(tUnitFrame->BU8));

    tUnitFrame->unitFrame = NULL;

    return tUnitFrame;
}

void updateTelemUnitData(telemetryUnitFrame *tUnitFrame, char *A1, char *A2, char *A3, char *A4, char *A5, char *B1, char *B2, char *B3, char *B4, char *B5, char *B6, char *B7, char *B8){


char *analogU[] = {tUnitFrame->AU1, tUnitFrame->AU2, tUnitFrame->AU3, tUnitFrame->AU4, tUnitFrame->AU5};
   
   
    char *digitalU[] = {tUnitFrame->BU1, tUnitFrame->BU2, tUnitFrame->BU3, tUnitFrame->BU4, tUnitFrame->BU5, tUnitFrame->BU6, tUnitFrame->BU7, tUnitFrame->BU8};
    char *analog[] = {A1, A2, A3, A4, A5};
    char *digital[] = {B1, B2, B3, B4, B5, B6, B7, B8};

    for (int i = 0; i < 5; i++)
    {
        strcpy(analogU[i], analog[i]);
        tUnitFrame->unitFrameSize += strlen(analog[i]);
    }

    for (int i = 0; i < 8; i++)
    {
        strcpy(digitalU[i], digital[i]);
        tUnitFrame->unitFrameSize += strlen(digital[i]);
    }

    printUnitFrameStructMembers(tUnitFrame);


}

void concatUnitData(telemetryUnitFrame *tUnitFrame)
{
    uint8_t *unitData = (uint8_t *)malloc((sizeof(uint8_t) * tUnitFrame->unitFrameSize + 1));

    snprintf((char *)unitData, tUnitFrame->unitFrameSize + 1,
             "%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
             tUnitFrame->typeIdentifier,
             tUnitFrame->AU1,
             tUnitFrame->AU2,
             tUnitFrame->AU3,
             tUnitFrame->AU4,
             tUnitFrame->AU5,
             tUnitFrame->BU1,
             tUnitFrame->BU2,
             tUnitFrame->BU3,
             tUnitFrame->BU4,
             tUnitFrame->BU5,
             tUnitFrame->BU6,
             tUnitFrame->BU7,
             tUnitFrame->BU8);

    tUnitFrame->unitFrame = (uint8_t *)unitData;
    printf("Concatenated Unit Data: %.*s\n", (int)tUnitFrame->unitFrameSize, tUnitFrame->unitFrame);
    printf("--------------------------------------------\n");
    return;
}

messageFrame *initMFrame(void)
{
    messageFrame *mFrame = (messageFrame *)malloc(sizeof(messageFrame));
    if (mFrame == NULL)
    {
        return NULL;
    } else {
        printf("Message Frame Initialized!\n");
    }

    mFrame->typeIdentifier = ':';
    memset(mFrame->addressee, 0, sizeof(mFrame->addressee));
    mFrame->endOfHeader = ':';
    mFrame->message = NULL;
    mFrame->messageSize = 0;

    return mFrame;
}

void updateMessageData(messageFrame *mFrame, char *addressee, char *message)
{
    strcpy(mFrame->addressee, addressee);
    mFrame->messageSize = strlen(message) + MESSAGE_HEADER_SIZE;
    if (mFrame->messageSize > 67)
    {
        mFrame->messageSize = 67;
        printf("Message Size Exceeded! Truncated to 67 characters\n");
    }
    mFrame->message = message;
    printMFrameStructMembers(mFrame);
}

void concatMessageData(messageFrame *mFrame)
{
    uint8_t *messageData = (uint8_t *)malloc((sizeof(uint8_t) * mFrame->messageSize + 1));

    snprintf((char *)messageData, mFrame->messageSize + 1,
             "%c%s%c%s",
             mFrame->typeIdentifier,
             mFrame->addressee,
             mFrame->endOfHeader,
             mFrame->message);

    mFrame->messageSize = strlen((char *)messageData) + 1;
    mFrame->messageFrame = (uint8_t *)messageData;

    printf("Concatenated Message Data: %.*s\n", (int)mFrame->messageSize, mFrame->messageFrame);
    printf("--------------------------------------------\n");
    return;
}

positionFrame *initPFrame(void)
{
    positionFrame *pFrame = (positionFrame *)malloc(sizeof(positionFrame));
    if (pFrame == NULL)
    {
        return NULL;
    } else {
        printf("Position Frame Initialized!\n");
    }

    pFrame->capability = '/'; // Simplex Capability
    pFrame->tableID = '/';
    memset(pFrame->time, 0, sizeof(pFrame->time));
    memset(pFrame->lat, 0, sizeof(pFrame->lat));
    memset(pFrame->lon, 0, sizeof(pFrame->lon));
    pFrame->SymCode = 'O';
    pFrame->comment = NULL;

    return pFrame; 
}

void updatePositionData(positionFrame *pFrame, char *time, char *lat, char *lon, char *comment)
{
    strcpy(pFrame->time, time);
    strcpy(pFrame->lat, lat);
    strcpy(pFrame->lon, lon);

    pFrame->comment = comment;

    if(pFrame->comment == NULL){
        pFrame->commentSize = 0;
    } else if(strlen(pFrame->comment) > 43){
        pFrame->commentSize = 43;
        printf("Comment Size Exceeded! Truncated to 43 characters\n");
    }
    else {
        pFrame->commentSize = strlen(comment);
    }
    pFrame->positionFrameSize = (pFrame->commentSize + POSITION_HEADER_SIZE); 
    printPFrameStructMembers(pFrame);
}

void concatPositionData(positionFrame *pFrame)
{
    uint8_t *positionData = (uint8_t *)malloc((sizeof(uint8_t) * pFrame->positionFrameSize  + 1));

    snprintf((char *)positionData, pFrame->positionFrameSize + 1,
             "%c%s%c%s%c%s%c%s",
             pFrame->capability,
             pFrame->time,
             pFrame->tableID,
             pFrame->lat,
             pFrame->SymCode,
             pFrame->lon,
             pFrame->SymCode,
             pFrame->comment);
    pFrame->positionFrameSize = strlen((char *)positionData) + 1;
    pFrame->positionFrame = (uint8_t *)positionData;
    printf("Concatenated Position Data: %.*s\n", (int)pFrame->positionFrameSize, pFrame->positionFrame);
    printf("--------------------------------------------\n");
    return;
}


void printPFrameStructMembers(positionFrame *pFrame)
{
    printf("Capability: %c\n", pFrame->capability);
    printf("Time: %s\n", pFrame->time);
    printf("Table ID: %c\n", pFrame->tableID);
    printf("Latitude: %s\n", pFrame->lat);
    printf("Longitude: %s\n", pFrame->lon);
    printf("Symbol Code: %c\n", pFrame->SymCode);
    printf("Comment: %s\n", pFrame->comment);
    printf("Comment Size: %ld\n", pFrame->commentSize);
}

void printMFrameStructMembers(messageFrame *mFrame)
{
    printf("Type Identifier: %c\n", mFrame->typeIdentifier);
    printf("Addressee: %.9s\n", mFrame->addressee);
    printf("End of Header: %c\n", mFrame->endOfHeader);
    printf("Message: %s\n", mFrame->message);
    printf("Message Size: %ld\n", mFrame->messageSize);
}

void printTFrameStructMembers(telemetryInfoFrame *tFrame)
{

    printf("Type Identifier: %c\n", tFrame->typeIdentifier);
    printf("Sequence Number: %.5s\n", tFrame->sequenceNumber);
    printf("Analog Value 1: %.4s\n", tFrame->analogValue1);
    printf("Analog Value 2: %.4s\n", tFrame->analogValue2);
    printf("Analog Value 3: %.4s\n", tFrame->analogValue3);
    printf("Analog Value 4: %.4s\n", tFrame->analogValue4);
    printf("Analog Value 5: %.4s\n", tFrame->analogValue5);
    printf("Digital Value: %.8s\n", tFrame->digitalValue);
    printf("Comment: %s\n", tFrame->comment);
    printf("Comment Length: %ld\n", tFrame->commentLength);
    printf("Telemetry Data: %s\n", tFrame->tData);
    printf("Telemetry Data Size: %ld\n", tFrame->tDataSize);
}

void printParamFrameStructMembers(telemetryParamFrame *tParamFrame)
{
    printf("Type Identifier: %.5s\n", tParamFrame->typeIdentifier);
    printf("A1: %.7s\n", tParamFrame->A1);
    printf("A2: %.7s\n", tParamFrame->A2);
    printf("A3: %.6s\n", tParamFrame->A3);
    printf("A4: %.6s\n", tParamFrame->A4);
    printf("A5: %.5s\n", tParamFrame->A5);
    printf("B1: %.6s\n", tParamFrame->B1);
    printf("B2: %.5s\n", tParamFrame->B2);
    printf("B3: %.4s\n", tParamFrame->B3);
    printf("B4: %.4s\n", tParamFrame->B4);
    printf("B5: %.4s\n", tParamFrame->B5);
    printf("B6: %.3s\n", tParamFrame->B6);
    printf("B7: %.3s\n", tParamFrame->B7);
    printf("B8: %.3s\n", tParamFrame->B8);
    printf("Param Frame Size: %ld\n", tParamFrame->paramFrameSize);
}

void printUnitFrameStructMembers(telemetryUnitFrame *tUnitFrame) {

    printf("Type Identifier: %.5s\n", tUnitFrame->typeIdentifier);
    printf("AU1: %.7s\n", tUnitFrame->AU1);
    printf("AU2: %.7s\n", tUnitFrame->AU2);
    printf("AU3: %.6s\n", tUnitFrame->AU3);
    printf("AU4: %.6s\n", tUnitFrame->AU4);
    printf("AU5: %.5s\n", tUnitFrame->AU5);
    printf("BU1: %.6s\n", tUnitFrame->BU1);
    printf("BU2: %.5s\n", tUnitFrame->BU2);
    printf("BU3: %.4s\n", tUnitFrame->BU3);
    printf("BU4: %.4s\n", tUnitFrame->BU4);
    printf("BU5: %.4s\n", tUnitFrame->BU5);
    printf("BU6: %.3s\n", tUnitFrame->BU6);
    printf("BU7: %.3s\n", tUnitFrame->BU7);
    printf("BU8: %.3s\n", tUnitFrame->BU8);
    printf("Unit Frame Size: %ld\n", tUnitFrame->unitFrameSize);
}
