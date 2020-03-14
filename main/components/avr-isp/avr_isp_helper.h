#include <stdlib.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#ifndef _OPTILOADER_H
#define _OPTILOADER_H

#define VERBOSE 1       // Debugging output?

#define FUSE_PROT 3			/* memory protection */
#define FUSE_LOW 0			/* Low fuse */
#define FUSE_HIGH 1			/* High fuse */
#define FUSE_EXT 2			/* Extended fuse */


#define PIN_NUM_MISO 25
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  19
#define PIN_NUM_CS   26
#define PIN_NUM_RST  18

#define LED_ERR 8
#define LED_PROGMODE 16

#define LOW 0
#define HIGH 1

#define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0])))

 #define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })
	 
	  #define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
	

// structure to hold signature and other relevant data about each chip
typedef struct {
   uint8_t sig [3];
   const char * desc;
   uint32_t flashSize;
   unsigned int baseBootSize;
   uint32_t pageSize;     // bytes
   uint8_t fuseWithBootloaderSize;  // ie. one of: lowFuse, highFuse, extFuse
   uint8_t timedWrites;    // if pollUntilReady won't work by polling the chip
} signatureType;

	
// actions to take
enum {
    checkFile,
    verifyFlash,
    writeToFlash,
};

// types of record in .hex file
enum {
    hexDataRecord,  // 00
    hexEndOfFile,   // 01
    hexExtendedSegmentAddressRecord, // 02
    hexStartSegmentAddressRecord,  // 03
    hexExtendedLinearAddressRecord, // 04
    hexStartLinearAddressRecord // 05
};	 

#define kb 1024
#define NO_FUSE 0xFF
#define NO_PAGE 0xFFFFFFFF

// meaning of bytes in above array
enum {
      lowFuse,
      highFuse,
      extFuse,
      lockByte,
      calibrationByte
};

// programming commands to send via SPI to the chip
enum {
	progamEnable = 0xAC,

	// writes are preceded by progamEnable
	chipErase = 0x80,
	writeLockByte = 0xE0,
	writeLowFuseByte = 0xA0,
	writeHighFuseByte = 0xA8,
	writeExtendedFuseByte = 0xA4,

	pollReady = 0xF0,

	programAcknowledge = 0x53,

	readSignatureByte = 0x30,
	readCalibrationByte = 0x38,

	readLowFuseByte = 0x50,       readLowFuseByteArg2 = 0x00,
	readExtendedFuseByte = 0x50,  readExtendedFuseByteArg2 = 0x08,
	readHighFuseByte = 0x58,      readHighFuseByteArg2 = 0x08,  
	readLockByte = 0x58,          readLockByteArg2 = 0x00,  

	readProgramMemory = 0x20,  
	writeProgramMemory = 0x4C,
	loadExtendedAddressByte = 0x4D,
	loadProgramMemory = 0x40,
    
};  // end of enum

uint32_t spi_transaction (spi_device_handle_t spi, uint8_t a, uint8_t b, uint8_t c, uint8_t d);
uint16_t readSignature (spi_device_handle_t spi);
bool programFuses (spi_device_handle_t spi, uint8_t *fuses);//
bool readFuses (spi_device_handle_t spi);//
void eraseChip(spi_device_handle_t spi);//
bool verifyImage (spi_device_handle_t spi, uint8_t *hextext);
void busyWait(spi_device_handle_t spi);//
uint8_t hexton (uint8_t h);//
bool verifyFuses (spi_device_handle_t spi, uint8_t *fuses, uint8_t *fusemask);//
void error(const char *string);
uint8_t program (spi_device_handle_t spi, const uint8_t b1, const uint8_t b2, const uint8_t b3, const uint8_t b4);
bool target_poweron (spi_device_handle_t spi);
bool start_pmode (spi_device_handle_t spi);
void showProgress();
uint8_t highByte(uint32_t data);
uint8_t lowByte(uint32_t data);
uint8_t readFlash (spi_device_handle_t spi, uint32_t addr);
void pollUntilReady (spi_device_handle_t spi);
void writeFlash (spi_device_handle_t spi, uint32_t addr, const uint8_t data);
void clearPage (spi_device_handle_t spi);
void commitPage (spi_device_handle_t spi, uint32_t addr);
void verifyData (spi_device_handle_t spi, const uint32_t addr, const uint8_t * pData, const int length);
void writeData (spi_device_handle_t spi, const uint32_t addr, const uint8_t * pData, const int length);
int getSignature (spi_device_handle_t spi);
void getFuseBytes (spi_device_handle_t spi, uint8_t * f);
bool processLine (spi_device_handle_t spi, const char * pLine, const uint8_t action, uint8_t count);
bool readHexFile (spi_device_handle_t spi, const char * fName, const uint8_t action);
void updateFuses(spi_device_handle_t spi, uint8_t *fuses, uint8_t *fusemask);
bool writeFlashContents (spi_device_handle_t spi, const char * name);
void stopProgramming(spi_device_handle_t spi);
int getLineNumber(void);

#endif
