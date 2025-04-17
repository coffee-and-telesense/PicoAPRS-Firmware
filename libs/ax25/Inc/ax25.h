#ifndef AX25_H
#define AX25_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define BIT_STREAM_LEN 256
#define CALL "KK7UUQ" // must be 6 characters
#define DEST "TEST  " // the spaces are important
#define SSID_DEST 0x60
#define SSID_SRC 0x61
#define CONTROL 0x03
#define PID 0xF0
#define FLAG 0x7E
#define CALL_LEN 6


typedef struct {
  uint8_t ssidDest;
  uint8_t ssidSrc;
  uint8_t cntrl;
  uint8_t pid;
  uint8_t flag;

  size_t infoSize;
  size_t crcFrameSize;
  size_t ax25FrameSize;
  size_t binAx25FrameSize;
  size_t binHdlcFrameSize;
  size_t nrziBinHdlcFrameSize;

  uint8_t *fcs;
  uint8_t *info;
  uint8_t *dest;
  uint8_t *src;
  uint8_t *crcFrame;
  uint8_t *ax25Frame;
  uint8_t *binAx25Frame;
  uint8_t *binHdlcFrame;
} iFrame;

// Structure to hold the NRZI bitstream and its size
typedef struct {
  uint8_t *nrziBinHdlcFrame;
  size_t size;
} hdlcFrame;

// Function prototypes
//----------------------------------------------------------------

//API User Functions
iFrame* initFrame(uint8_t *info, size_t infoSize);
hdlcFrame processFrame(iFrame *frame);
hdlcFrame processFrameVerbose(iFrame *frame);

//----------------------------------------------------------------


void cleanFrame(iFrame *frame);
void shiftBits(iFrame *frame);
void printHex(const char *label, uint8_t *data, size_t len);
void makeCRC(iFrame *frame);
void concatCrcFrame(iFrame *frame);
void concatAx25Frame(iFrame *frame);
void hextobin_rev(iFrame *frame);
void printBin(const char *label, uint8_t *bitstream, size_t len);
void bitStuff(iFrame *frame);
uint8_t *genNRZI(iFrame *frame);
#endif // AX25_H
