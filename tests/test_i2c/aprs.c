#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "aprs.h"
#include "aprsConfig.h"
#include "ax25.h"

// --- Telemetry Info Frame ---
telemetryInfoFrame *initTFrame(void)
{
    telemetryInfoFrame *tFrame = malloc(sizeof(*tFrame));
    if (!tFrame) return NULL;
    printf("Telemetry Frame Initialized!\n");
    tFrame->typeIdentifier = 'T';
    memset(tFrame->sequenceNumber, 0, sizeof tFrame->sequenceNumber);
    memset(tFrame->analogValue1,   0, sizeof tFrame->analogValue1);
    memset(tFrame->analogValue2,   0, sizeof tFrame->analogValue2);
    memset(tFrame->analogValue3,   0, sizeof tFrame->analogValue3);
    memset(tFrame->analogValue4,   0, sizeof tFrame->analogValue4);
    memset(tFrame->analogValue5,   0, sizeof tFrame->analogValue5);
    memset(tFrame->digitalValue,   0, sizeof tFrame->digitalValue);
    tFrame->comment = NULL;
    tFrame->commentLength = 0;
    tFrame->tData = NULL;
    tFrame->tDataSize = 0;
    return tFrame;
}

void updateTelemData(
    telemetryInfoFrame *tFrame,
    uint8_t *analog1, uint8_t *analog2, uint8_t *analog3,
    uint8_t *analog4, uint8_t *analog5,
    uint8_t *digital,
    char *comment)
{
    memcpy(tFrame->sequenceNumber, SEQUENCE_NUMBER, sizeof tFrame->sequenceNumber);
    memcpy(tFrame->analogValue1,   analog1, sizeof tFrame->analogValue1);
    memcpy(tFrame->analogValue2,   analog2, sizeof tFrame->analogValue2);
    memcpy(tFrame->analogValue3,   analog3, sizeof tFrame->analogValue3);
    memcpy(tFrame->analogValue4,   analog4, sizeof tFrame->analogValue4);
    memcpy(tFrame->analogValue5,   analog5, sizeof tFrame->analogValue5);
    memcpy(tFrame->digitalValue,   digital, sizeof tFrame->digitalValue);
    tFrame->comment = comment;
    tFrame->commentLength = strlen(comment);
    tFrame->tData = NULL;
    tFrame->tDataSize = 0;
}

void concatTelemData(telemetryInfoFrame *tFrame)
{
    size_t bufSize = TELEM_DATA_SIZE + tFrame->commentLength + 1;
    char *buf = malloc(bufSize);
    if (!buf) return;
    snprintf(buf, bufSize,
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
    tFrame->tData     = (uint8_t*)buf;
    tFrame->tDataSize = TELEM_DATA_SIZE + tFrame->commentLength;
    printf("Concatenated Telemetry Data: %.*s\n",
           (int)tFrame->tDataSize, buf);
    printTFrameStructMembers(tFrame);
    //free(buf); // optional, but keep tFrame->tData separately if needed
}

// --- Telemetry Parameter Frame ---
telemetryParamFrame *initParamFrame(void)
{
    telemetryParamFrame *f = malloc(sizeof *f);
    if (!f) return NULL;
    printf("Parameter Frame Initialized!\n");
    strcpy(f->typeIdentifier, "PARM.");
    f->paramFrameSize = PARAM_IDENTIFIER_SIZE;
    memset(f->A1, 0, sizeof f->A1);
    memset(f->A2, 0, sizeof f->A2);
    memset(f->A3, 0, sizeof f->A3);
    memset(f->A4, 0, sizeof f->A4);
    memset(f->A5, 0, sizeof f->A5);
    memset(f->B1, 0, sizeof f->B1);
    memset(f->B2, 0, sizeof f->B2);
    memset(f->B3, 0, sizeof f->B3);
    memset(f->B4, 0, sizeof f->B4);
    memset(f->B5, 0, sizeof f->B5);
    memset(f->B6, 0, sizeof f->B6);
    memset(f->B7, 0, sizeof f->B7);
    memset(f->B8, 0, sizeof f->B8);
    f->paramFrame = NULL;
    return f;
}

void updateTelemParamData(
    telemetryParamFrame *f,
    char *A1,char *A2,char *A3,char *A4,char *A5,
    char *B1,char *B2,char *B3,char *B4,
    char *B5,char *B6,char *B7,char *B8)
{
    char *analog[5]  = {f->A1,f->A2,f->A3,f->A4,f->A5};
    char *digital[8] = {f->B1,f->B2,f->B3,f->B4,f->B5,f->B6,f->B7,f->B8};
    char *inA[5]     = {A1,A2,A3,A4,A5};
    char *inD[8]     = {B1,B2,B3,B4,B5,B6,B7,B8};
    for(int i=0;i<5;i++){ strcpy(analog[i],inA[i]); f->paramFrameSize += strlen(inA[i]); }
    for(int i=0;i<8;i++){ strcpy(digital[i],inD[i]); f->paramFrameSize += strlen(inD[i]); }
    printParamFrameStructMembers(f);
}

void concatParamData(telemetryParamFrame *f)
{
    size_t total = f->paramFrameSize + 1;
    char *buf = malloc(total);
    if(!buf) return;
    snprintf(buf,total,
             "%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
             f->typeIdentifier,
             f->A1,f->A2,f->A3,f->A4,f->A5,
             f->B1,f->B2,f->B3,f->B4,f->B5,f->B6,f->B7,f->B8);
    printf("Concatenated Param Data: %.*s\n", (int)f->paramFrameSize, buf);
    //free(buf);
}

// --- Telemetry Unit Frame ---
telemetryUnitFrame *initUnitFrame(void)
{
    telemetryUnitFrame *u = malloc(sizeof *u);
    if(!u) return NULL;
    printf("Unit Frame Initialized!\n");
    strcpy(u->typeIdentifier,"UNIT.");
    u->unitFrameSize = UNIT_IDENTIFIER_SIZE;
    memset(u->AU1,0,sizeof u->AU1);
    memset(u->AU2,0,sizeof u->AU2);
    memset(u->AU3,0,sizeof u->AU3);
    memset(u->AU4,0,sizeof u->AU4);
    memset(u->AU5,0,sizeof u->AU5);
    memset(u->BU1,0,sizeof u->BU1);
    memset(u->BU2,0,sizeof u->BU2);
    memset(u->BU3,0,sizeof u->BU3);
    memset(u->BU4,0,sizeof u->BU4);
    memset(u->BU5,0,sizeof u->BU5);
    memset(u->BU6,0,sizeof u->BU6);
    memset(u->BU7,0,sizeof u->BU7);
    memset(u->BU8,0,sizeof u->BU8);
    return u;
}

void updateTelemUnitData(
    telemetryUnitFrame *u,
    char *A1,char *A2,char *A3,char *A4,char *A5,
    char *B1,char *B2,char *B3,char *B4,
    char *B5,char *B6,char *B7,char *B8)
{
    char *an[5] = {u->AU1,u->AU2,u->AU3,u->AU4,u->AU5};
    char *di[8] = {u->BU1,u->BU2,u->BU3,u->BU4,u->BU5,u->BU6,u->BU7,u->BU8};
    char *inA[5] = {A1,A2,A3,A4,A5};
    char *inD[8] = {B1,B2,B3,B4,B5,B6,B7,B8};
    for(int i=0;i<5;i++){ strcpy(an[i],inA[i]); u->unitFrameSize += strlen(inA[i]); }
    for(int i=0;i<8;i++){ strcpy(di[i],inD[i]); u->unitFrameSize += strlen(inD[i]); }
    printUnitFrameStructMembers(u);
}

void concatUnitData(telemetryUnitFrame *u)
{
    size_t total = u->unitFrameSize + 1;
    char *buf = malloc(total);
    if(!buf) return;
    snprintf(buf,total,
             "%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
             u->typeIdentifier,
             u->AU1,u->AU2,u->AU3,u->AU4,u->AU5,
             u->BU1,u->BU2,u->BU3,u->BU4,u->BU5,u->BU6,u->BU7,u->BU8);
    printf("Concatenated Unit Data: %.*s\n", (int)u->unitFrameSize, buf);
    //free(buf);
}

// --- Message Frame ---
messageFrame *initMFrame(void)
{
    messageFrame *m = malloc(sizeof *m);
    if(!m) return NULL; printf("Message Frame Initialized!\n");
    m->typeIdentifier = ':';
    memset(m->addressee,0,sizeof m->addressee);
    m->endOfHeader = ':';
    m->message = NULL; m->messageSize = 0;
    return m;
}

void updateMessageData(messageFrame *m, char *addr, char *msg)
{
    strcpy(m->addressee,addr);
    m->messageSize = strlen(msg)+MESSAGE_HEADER_SIZE;
    if(m->messageSize>67){ m->messageSize=67; printf("Message truncated\n"); }
    m->message = msg;
    printMFrameStructMembers(m);
}

void concatMessageData(messageFrame *m)
{
    size_t total = m->messageSize+1;
    char *buf = malloc(total);
    if(!buf) return;
    snprintf(buf,total,"%c%s%c%s",
             m->typeIdentifier,m->addressee,m->endOfHeader,m->message);
    printf("Concatenated Message Data: %.*s\n", (int)m->messageSize, buf);
    //free(buf);
}

// --- Position Frame ---
positionFrame *initPFrame(void)
{
    positionFrame *p = malloc(sizeof *p);
    if(!p) return NULL; printf("Position Frame Initialized!\n");
    p->capability='/';p->tableID='/';
    memset(p->time,0,sizeof p->time);
    memset(p->lat,0,sizeof p->lat);
    memset(p->lon,0,sizeof p->lon);
    p->SymCode='O';p->comment=NULL;p->commentSize=0;
    return p;
}

void updatePositionData(positionFrame *p,char *time,char *lat,char *lon,char *cmt)
{
    strcpy(p->time,time);strcpy(p->lat,lat);strcpy(p->lon,lon);
    p->comment=cmt; p->commentSize = cmt? strnlen(cmt,43):0;
    p->positionFrameSize=p->commentSize+POSITION_HEADER_SIZE;
    printPFrameStructMembers(p);
}

void concatPositionData(positionFrame *p)
{
    size_t total=p->positionFrameSize+1;
    char *buf=malloc(total); if(!buf)return;
    snprintf(buf,total,"%c%s%c%s%c%s%c%s",
             p->capability,p->time,p->tableID,
             p->lat,p->SymCode,p->lon,p->SymCode,p->comment? p->comment:"");
    printf("Concatenated Position Data: %.*s\n",(int)p->positionFrameSize,buf);
    //free(buf);
}

// --- Print Struct Members ---
void printTFrameStructMembers(telemetryInfoFrame *f)
{
    printf("Type Identifier: %c\n",f->typeIdentifier);
    printf("Sequence Number: %.5s\n",f->sequenceNumber);
    printf("Analog1: %.4s\n",f->analogValue1);
    printf("Analog2: %.4s\n",f->analogValue2);
    printf("Analog3: %.4s\n",f->analogValue3);
    printf("Analog4: %.4s\n",f->analogValue4);
    printf("Analog5: %.4s\n",f->analogValue5);
    printf("Digital Value: %.8s\n",f->digitalValue);
    printf("Comment Length: %zu\n",f->commentLength);
}

void printParamFrameStructMembers(telemetryParamFrame *f)
{
    printf("Param Type ID: %.5s\n",f->typeIdentifier);
    printf("A1: %.7s A2: %.7s A3: %.6s A4: %.6s A5: %.5s\n",
           f->A1,f->A2,f->A3,f->A4,f->A5);
    printf("B1: %.6s B2: %.5s B3: %.4s B4: %.4s B5: %.4s B6: %.3s B7: %.3s B8: %.3s\n",
           f->B1,f->B2,f->B3,f->B4,f->B5,f->B6,f->B7,f->B8);
}

void printUnitFrameStructMembers(telemetryUnitFrame *u)
{
    printf("Unit Type ID: %.5s\n",u->typeIdentifier);
    printf("AU1: %.7s AU2: %.7s AU3: %.6s AU4: %.6s AU5: %.5s\n",
           u->AU1,u->AU2,u->AU3,u->AU4,u->AU5);
    printf("BU1: %.6s BU2: %.5s BU3: %.4s BU4: %.4s BU5: %.4s BU6: %.3s BU7: %.3s BU8: %.3s\n",
           u->BU1,u->BU2,u->BU3,u->BU4,u->BU5,u->BU6,u->BU7,u->BU8);
}

void printMFrameStructMembers(messageFrame *m)
{
    printf("Msg Type ID: %c Addressee: %.9s MessageSize: %zu\n",
           m->typeIdentifier,m->addressee,m->messageSize);
}

void printPFrameStructMembers(positionFrame *p)
{
    printf("Pos Cap:%c Time:%s TableID:%c Lat:%s Lon:%s Sym:%c CommentSize:%zu\n",
           p->capability,p->time,p->tableID,p->lat,p->lon,p->SymCode,p->commentSize);
}
