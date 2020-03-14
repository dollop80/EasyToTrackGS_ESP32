#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/unistd.h>
#include <sys/stat.h>
//#include "mbedtls/md5.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "avr_isp_helper.h"

static const char *TAG = "AVR_ISP";


// see Atmega datasheets
const signatureType signatures [] = 
  {
//     signature        description   flash size   bootloader  flash  fuse
//                                                     size    page    to
//                                                             size   change

  // Attiny84 family
  { { 0x1E, 0x91, 0x0B }, "ATtiny24",   2 * kb,           0,   32,   NO_FUSE, false },
  { { 0x1E, 0x92, 0x07 }, "ATtiny44",   4 * kb,           0,   64,   NO_FUSE, false },
  { { 0x1E, 0x93, 0x0C }, "ATtiny84",   8 * kb,           0,   64,   NO_FUSE, false },

  // Attiny85 family
  { { 0x1E, 0x91, 0x08 }, "ATtiny25",   2 * kb,           0,   32,   NO_FUSE, false },
  { { 0x1E, 0x92, 0x06 }, "ATtiny45",   4 * kb,           0,   64,   NO_FUSE, false },
  { { 0x1E, 0x93, 0x0B }, "ATtiny85",   8 * kb,           0,   64,   NO_FUSE, false },

  // Atmega328 family
  { { 0x1E, 0x92, 0x0A }, "ATmega48PA",   4 * kb,         0,    64,  NO_FUSE, false },
  { { 0x1E, 0x93, 0x0F }, "ATmega88PA",   8 * kb,       256,   128,  extFuse, false },
  { { 0x1E, 0x94, 0x0B }, "ATmega168PA", 16 * kb,       256,   128,  extFuse, false },
  { { 0x1E, 0x94, 0x06 }, "ATmega168",   16 * kb,       256,   128,  extFuse, false },
  { { 0x1E, 0x95, 0x0F }, "ATmega328P",  32 * kb,       512,   128,  highFuse, false },

  // Atmega644 family
  { { 0x1E, 0x94, 0x0A }, "ATmega164P",   16 * kb,      256,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x08 }, "ATmega324P",   32 * kb,      512,   128,  highFuse, false },
  { { 0x1E, 0x96, 0x0A }, "ATmega644P",   64 * kb,   1 * kb,   256,  highFuse, false },

  // Atmega2560 family
  { { 0x1E, 0x96, 0x08 }, "ATmega640",    64 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x97, 0x03 }, "ATmega1280",  128 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x97, 0x04 }, "ATmega1281",  128 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x98, 0x01 }, "ATmega2560",  256 * kb,   1 * kb,   256,  highFuse, false },
      
  { { 0x1E, 0x98, 0x02 }, "ATmega2561",  256 * kb,   1 * kb,   256,  highFuse, false },
  
  // AT90USB family
  { { 0x1E, 0x93, 0x82 }, "At90USB82",    8 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x94, 0x82 }, "At90USB162",  16 * kb,       512,   128,  highFuse, false },

  // Atmega32U2 family
  { { 0x1E, 0x93, 0x89 }, "ATmega8U2",    8 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x94, 0x89 }, "ATmega16U2",  16 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x8A }, "ATmega32U2",  32 * kb,       512,   128,  highFuse, false },

  // Atmega32U4 family -  (datasheet is wrong about flash page size being 128 words)
  { { 0x1E, 0x94, 0x88 }, "ATmega16U4",  16 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x87 }, "ATmega32U4",  32 * kb,       512,   128,  highFuse, false },

  // ATmega1284P family
  { { 0x1E, 0x97, 0x05 }, "ATmega1284P", 128 * kb,   1 * kb,   256,  highFuse , false },
  
  // ATtiny4313 family
  { { 0x1E, 0x91, 0x0A }, "ATtiny2313A",   2 * kb,        0,    32,  NO_FUSE, false },
  { { 0x1E, 0x92, 0x0D }, "ATtiny4313",    4 * kb,        0,    64,  NO_FUSE, false },

  // ATtiny13 family
  { { 0x1E, 0x90, 0x07 }, "ATtiny13A",     1 * kb,        0,    32,  NO_FUSE, false },
 
   // Atmega8A family
  { { 0x1E, 0x93, 0x07 }, "ATmega8A",      8 * kb,      256,    64,  highFuse, true },

  // ATmega64rfr2 family
  { { 0x1E, 0xA6, 0x02 }, "ATmega64rfr2",  256 * kb, 1 * kb,   256, highFuse, false },
  { { 0x1E, 0xA7, 0x02 }, "ATmega128rfr2", 256 * kb, 1 * kb,   256, highFuse, false },
  { { 0x1E, 0xA8, 0x02 }, "ATmega256rfr2", 256 * kb, 1 * kb,   256, highFuse, false },

  };  // end of signatures



// copy of fuses/lock bytes found for this processor
uint8_t fuses [5];

    // Normal fuses, written after writing to flash (but before
    // verifying). Fuses set to zero are untouched.
//uint8_t wfuses[4]  =  {0xFF, 0xDD, 0xF8, 0x0F}; // {low, high, extended, lock}
uint8_t wfuses[4]  =  {0x3E, 0xC2, 0x00, 0x00}; // {low, high, extended, lock}
    // Fuse verify mask. Any bits set to zero in these values are
    // ignored while verifying the fuses after writing them. All (and
    // only) bits that are unused for this atmega chip should be zero
    // here.
uint8_t fuse_mask[4]  =  {0xFF, 0xFF, 0x00, 0x00}; // {low, high, extended, lock}


// copy of current signature entry for matching processor
signatureType currentSignature;

uint8_t pageBuffer[128];		       /* One page of flash */

bool gotEndOfFile;
uint32_t extendedAddress;

uint32_t lowestAddress;
uint32_t highestAddress;
uint32_t bytesWritten;
unsigned int lineCount;
uint8_t lastAddressMSB = 0;
uint32_t pagesize;
uint32_t pagemask;
uint32_t oldPage;
int lineNumber = 0;

int getLineNumber()
{
	return lineNumber;
}

/*
 * programmingFuses
 * Program the fuse/lock bits
 */
bool programFuses (spi_device_handle_t spi, uint8_t *fuses)
{
    
  uint8_t f;
  esp_err_t ret;
  printf("\nSetting fuses ");
  
  //We work with fuses on low SPI speed
  ret=spi_bus_remove_device(spi);
  ESP_ERROR_CHECK(ret);
  
  spi_device_interface_config_t devcfg={
	.clock_speed_hz=1*1000*10,           //Clock out at 10 MHz
	.mode=0,                                //SPI mode 0
	.spics_io_num=PIN_NUM_CS,               //CS pin
	.queue_size=7,                          //We want to be able to queue 7 transactions at a time
    };
	
	//Attach the programmer to the SPI bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);

  busyWait(spi);
  f = (fuses[FUSE_LOW]);
  if (f) {
    printf("\n\rSet Low Fuse to: ");
    printf("0x%02X",f);
    printf(" -> ");
    //SPI.beginTransaction(fuses_spisettings);
    printf("0x%02X ",spi_transaction(spi, 0xAC, 0xA0, 0x00, f));
    //SPI.endTransaction();
  }
  busyWait(spi);
  f = (fuses[FUSE_HIGH]);
  if (f) {
    printf("\n\rSet High Fuse to: ");
    printf("0x%02X",f);
    printf(" -> ");
    //SPI.beginTransaction(fuses_spisettings);
    printf("0x%02X ",spi_transaction(spi, 0xAC, 0xA8, 0x00, f));
    //SPI.endTransaction();
  }
  busyWait(spi);
  f = (fuses[FUSE_EXT]);
  if (f) {
    printf("\n\rSet Ext Fuse to: ");
    printf("0x%02X",f);
    printf(" -> ");
    //SPI.beginTransaction(fuses_spisettings);
    printf("0x%02X ",spi_transaction(spi, 0xAC, 0xA4, 0x00, f));
    //SPI.endTransaction();
  }
  busyWait(spi);
  f = (fuses[FUSE_PROT]);
  if (f) {
    printf("\n\rSet Lock Fuse to: ");
    printf("0x%02X",f);
    printf(" -> ");
    //SPI.beginTransaction(fuses_spisettings);
    printf("0x%02X ",spi_transaction(spi, 0xAC, 0xE0, 0x00, f));
    //SPI.endTransaction();
  }
  busyWait(spi);
  printf("\r\n");
  
  //Switch back to normal SPI speed
  ret=spi_bus_remove_device(spi);
  ESP_ERROR_CHECK(ret);
  
  devcfg.clock_speed_hz=1*1000*200;

	
	//Attach the programmer to the SPI bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
    
  return true;			/* */
}

/*
 * verifyFuses
 * Verifies a fuse set
 */
bool verifyFuses (spi_device_handle_t spi, uint8_t *fuses, uint8_t *fusemask)
{
  uint8_t f;

  printf("\n\rVerifying fuses...");
  /*
  f = (fuses[FUSE_PROT]);
  if (f) {
    //SPI.beginTransaction(fuses_spisettings);
    uint8_t readfuse = spi_transaction(spi, 0x58, 0x00, 0x00, 0x00);  // lock fuse
    //SPI.endTransaction();

    readfuse &= (fusemask[FUSE_PROT]);
    printf("\n\rLock Fuse: 0x%02X is 0x%02X", f, readfuse);
    if (readfuse != f) 
      return false;
  }
  */
  f = (fuses[FUSE_LOW]);
  if (f) {
    //SPI.beginTransaction(fuses_spisettings);
    uint8_t readfuse = spi_transaction(spi, 0x50, 0x00, 0x00, 0x00);  // low fuse
    //SPI.endTransaction();
        
    printf("\n\rLow Fuse: 0x%02X is 0x%02X", f, readfuse);
    readfuse &= (fusemask[FUSE_LOW]);
    if (readfuse != f) 
      return false;
  }
  f = (fuses[FUSE_HIGH]);
  if (f) {
    //SPI.beginTransaction(fuses_spisettings);
    uint8_t readfuse = spi_transaction(spi, 0x58, 0x08, 0x00, 0x00);  // high fuse
    //SPI.endTransaction();
        
    readfuse &= (fusemask[FUSE_HIGH]);
    printf("\n\rHigh Fuse: 0x%02X is 0x%02X", f, readfuse);
    if (readfuse != f) 
      return false;
  }
  f = (fuses[FUSE_EXT]);
  if (f) {
    //SPI.beginTransaction(fuses_spisettings);
    uint8_t readfuse = spi_transaction(spi, 0x50, 0x08, 0x00, 0x00);  // ext fuse
    //SPI.endTransaction();
    
    readfuse &= (fusemask[FUSE_EXT]);
    printf("\n\rExt Fuse: 0x%02X is 0x%02X\n\r", f, readfuse);
    if (readfuse != f) 
      return false;
  }
  printf("\r\n");
  
  return true;			/* */
}


/*
 * readFuses
 * Reads a fuse set
 */
bool readFuses(spi_device_handle_t spi)
{
  //uint8_t f;

  printf("Verifying fuses...");

    uint8_t readfuse = spi_transaction(spi, 0x58, 0x00, 0x00, 0x00);  // lock fuse
    printf("\tLock Fuse: 0x%2x ", readfuse);

    readfuse = spi_transaction(spi, 0x50, 0x00, 0x00, 0x00);  // low fuse        
    printf("\tLow Fuse: 0x%2x ", readfuse);

    readfuse = spi_transaction(spi, 0x58, 0x08, 0x00, 0x00);  // high fuse
    printf("\tHigh Fuse: 0x%2x ", readfuse);

    readfuse = spi_transaction(spi, 0x50, 0x08, 0x00, 0x00);  // ext fuse
    printf("\tExt Fuse: 0x%2x ", readfuse);

  printf("\r\n");
  
  return true;
}



// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
//  processor may return a result on the 4th transfer, this is returned.
uint8_t program (spi_device_handle_t spi, const uint8_t b1, const uint8_t b2, const uint8_t b3, const uint8_t b4)
  {
#if 1
  uint8_t r;
  esp_err_t ret;
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&b1;               //The data is the cmd itself
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
   
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&b2;               //The data is the cmd itself
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.

  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&b3;               //The data is the cmd itself
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.

  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&b4;               //The data is the cmd itself
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
  r = *(uint8_t*) t.rx_data;
#else
	portDISABLE_INTERRUPTS(); 
  BB_SPITransfer (b1);  
  BB_SPITransfer (b2);  
  BB_SPITransfer (b3);  
  uint8_t r = BB_SPITransfer (b4);  
  portENABLE_INTERRUPTS(); 
#endif
  
  return r;
  } // end of program




// Send the erase command, then busy wait until the chip is erased

void eraseChip(spi_device_handle_t spi) {
  //SPI.beginTransaction(fuses_spisettings);
   uint32_t ret = spi_transaction(spi, 0xAC, 0x80, 0, 0);	// chip erase
   if((ret&0x00FFF000) == 0xAC8000)
		printf("Erase successful\n\r");
   else
	   printf("Erase failed\n\r");
  //SPI.endTransaction();

  busyWait(spi);
}

// Simply polls the chip until it is not busy any more - for erasing and programming
void busyWait(spi_device_handle_t spi)  {
  //SPI.beginTransaction(fuses_spisettings);
  uint8_t busybit;
  do {
    busybit = spi_transaction(spi, 0xF0, 0x0, 0x0, 0x0);
    printf("%d ", busybit);
  } while (busybit & 0x01);
  //SPI.endTransaction();
}


/*
 * Functions specific to ISP programming of an AVR
 */
uint32_t spi_transaction (spi_device_handle_t spi, uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  uint32_t n, m, r;
  #if 1
  esp_err_t ret;
  //SPI.transfer(a);
  spi_transaction_t t;
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&a;               //The data is the cmd itself
  //t.user=(void*)0;                //D/C needs to be set to 0
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
   
  //n = SPI.transfer(b);
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&b;               //The data is the cmd itself
  //t.user=(void*)0;                //D/C needs to be set to 0
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
  n = *(uint32_t*) t.rx_data;

  //if (n != a) error = -1;
  //m = SPI.transfer(c);
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&c;               //The data is the cmd itself
  //t.user=(void*)0;                //D/C needs to be set to 0
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
  m = *(uint32_t*) t.rx_data;

  //r = SPI.transfer(d);
  memset(&t, 0, sizeof(t));       //Zero out the transaction
  t.length=8;                     //Command is 8 bits
  t.tx_buffer=&d;               //The data is the cmd itself
  //t.user=(void*)0;                //D/C needs to be set to 0
  t.flags = SPI_TRANS_USE_RXDATA;
  ret=spi_device_polling_transmit(spi, &t);  //Transmit!
  assert(ret==ESP_OK);            //Should have had no issues.
  r = *(uint32_t*) t.rx_data;

#else
	portDISABLE_INTERRUPTS(); 
  BB_SPITransfer (a);  
  n = BB_SPITransfer (b);  
  m = BB_SPITransfer (c);  
  r = BB_SPITransfer (d);  
  portENABLE_INTERRUPTS(); 
#endif
  return 0xFFFFFF & (((uint32_t)n<<16)+(m<<8) + r);
}


/*
 * hexton
 * Turn a Hex digit (0..9, A..F) into the equivalent binary value (0-16)
 */
uint8_t hexton (uint8_t h)
{
  if (h >= '0' && h <= '9')
    return(h - '0');
  if (h >= 'A' && h <= 'F')
    return((h - 'A') + 10);
  printf("Bad hex digit! %d\n\r", h);
	return -1;
}


void showProgress()
{
	//printf(".....");
}

uint8_t highByte(uint32_t data)
{
	return (data>>8) & 0xFF;
}

uint8_t lowByte(uint32_t data)
{
	return (data & 0xFF);
}

// read a byte from flash memory
uint8_t readFlash (spi_device_handle_t spi, uint32_t addr)
  {
  uint8_t high = ((addr & 1) == 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  uint8_t MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
    {
    program(spi, loadExtendedAddressByte, 0, MSB , 0); 
    lastAddressMSB = MSB;
    }  // end if different MSB

  return program (spi, readProgramMemory | high, highByte (addr), lowByte (addr), 0);
  } // end of readFlash


// count errors
unsigned int errors;
  
  
  // poll the target device until it is ready to be programmed
void pollUntilReady (spi_device_handle_t spi)
  {
  if (currentSignature.timedWrites)
    vTaskDelay(50 / portTICK_RATE_MS);  // at least 2 x WD_FLASH which is 4.5 mS
  else
    {  
    while ((program (spi, pollReady, 0, 0, 0) & 1) == 1)
      {}  // wait till ready  
    }  // end of if
  }  // end of pollUntilReady
  
// write a byte to the flash memory buffer (ready for committing)
void writeFlash (spi_device_handle_t spi, uint32_t addr, const uint8_t data)
  {
  uint8_t high = ((addr & 1) == 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  program (spi, loadProgramMemory | high, 0, lowByte (addr), data);
  } // end of writeFlash  
  
  
  // clear entire temporary page to 0xFF in case we don't write to all of it 
void clearPage (spi_device_handle_t spi)
{
  unsigned int len = currentSignature.pageSize;
  for (unsigned int i = 0; i < len; i++)
    writeFlash (spi, i, 0xFF);
}  // end of clearPage
  
// commit page to flash memory
void commitPage (spi_device_handle_t spi, uint32_t addr)
  {
  //printf("commiting page %d\n\r", addr);
	  
  addr >>= 1;  // turn into word address
  
  // set the extended (most significant) address byte if necessary
  uint8_t MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
    {
    program (spi, loadExtendedAddressByte, 0, MSB, 0); 
    lastAddressMSB = MSB;
    }  // end if different MSB
      
  program (spi, writeProgramMemory, highByte (addr), lowByte (addr), 0);
  pollUntilReady (spi); 
  
  clearPage(spi);  // clear ready for next page full
  }  // end of commitPage
  
  
void verifyData (spi_device_handle_t spi, const uint32_t addr, const uint8_t * pData, const int length)
  {
  // check each byte
  for (int i = 0; i < length; i++)
    {
    uint32_t thisPage = (addr + i) & pagemask;
    // page changed? show progress
    if (thisPage != oldPage && oldPage != NO_PAGE){
		//printf("PAGE CHANGED TO: %d\n\r", thisPage);
		printf(".");
	}
    // now this is the current page
    oldPage = thisPage;
	
    uint8_t found = readFlash (spi, addr + i);
    uint8_t expected = pData [i];
	//if(i%8==0)
	//	printf("\r\n");
	
    if (found != expected)
	{
      errors++;	  
	  	printf("%02X!=%02X ", expected, found);
	}
	else
	{
		//printf("%02X==%02X ", expected, found);
	}

    }  // end of for
    
  }  // end of verifyData
  
  
  // write data to temporary buffer, ready for committing  
void writeData (spi_device_handle_t spi, const uint32_t addr, const uint8_t * pData, const int length)
  {
  // write each byte
  for (int i = 0; i < length; i++)
    {
    uint32_t thisPage = (addr + i) & pagemask;
    // page changed? commit old one
    if (thisPage != oldPage && oldPage != NO_PAGE){
      commitPage (spi, oldPage);
      //printf("PAGE CHANGED TO: %d\n\r", thisPage);
	  printf(".");
	}
    // now this is the current page
    oldPage = thisPage;
	
    // put byte into work buffer
    writeFlash (spi, addr + i, pData [i]);
    }  // end of for
    
  }  // end of writeData
  
  int getSignature (spi_device_handle_t spi)
  {
  int foundSig = -1;
  lastAddressMSB = 0;
  uint8_t sig [3];
  for (uint8_t i = 0; i < 3; i++)
    {
    sig [i] = program (spi, readSignatureByte, 0, i, 0); 
    }  // end for each signature byte
  
  for (unsigned int j = 0; j < NUMITEMS (signatures); j++)
    {
    memcpy (&currentSignature, &signatures [j], sizeof currentSignature);
	
    if (memcmp (sig, signatures[j].sig, sizeof sig) == 0)
      {
      foundSig = j;
      // make sure extended address is zero to match lastAddressMSB variable
      program (spi, loadExtendedAddressByte, 0, 0, 0); 
      }  // end of signature found
    }  // end of for each signature
	
  if(foundSig!=-1)
	  printf ("chip found: %d\n\r", foundSig);
  else
	printf ("MSG_UNRECOGNIZED_SIGNATURE");
  return foundSig;
  }  // end of getSignature
  
void getFuseBytes (spi_device_handle_t spi, uint8_t * f)
  {
  fuses [lowFuse]   = program (spi, readLowFuseByte, readLowFuseByteArg2, 0, 0);
  fuses [highFuse]  = program (spi, readHighFuseByte, readHighFuseByteArg2, 0, 0);
  fuses [extFuse]   = program (spi, readExtendedFuseByte, readExtendedFuseByteArg2, 0, 0);
  fuses [lockByte]  = program (spi, readLockByte, readLockByteArg2, 0, 0);
  fuses [calibrationByte]  = program (spi, readCalibrationByte, 0, 0, 0);
  
	for(int i = 0; i<sizeof(fuses); i++)
	{
		f[i]=fuses[i];
	}
  }  // end of getFuseBytes
  
  
// returns true if error, false if OK
 bool processLine (spi_device_handle_t spi, const char * pLine, const uint8_t action, uint8_t count)
  {
	  
	  uint8_t b;
  if (*pLine++ != ':')
     {
     printf("MSG_LINE_DOES_NOT_START_WITH_COLON\n\r");
     return true;  // error
     } 
  
  const int maxHexData = 46;
  uint8_t hexBuffer [maxHexData];
  int bytesInLine = 0;
  
  if (action == checkFile)
    if (lineCount++ % 40 == 0)
      showProgress ();
    
  //printf("%s\n\r", pLine);
  // convert entire line from ASCII into binary
  while (count)
    {
		
    // can't fit?
    if (bytesInLine >= maxHexData)
      {
      printf("MSG_LINE_TOO_LONG\n\r");
      return true;
      } // end if too long

    b = hexton(*pLine++);  
    b = (b<<4) + hexton(*pLine++);
	hexBuffer[bytesInLine++] = b;
	count -=2;
	}  // end of while
    
  if (bytesInLine < 5)
    {
    printf("MSG_LINE_TOO_SHORT\n\r");
    return true;  
    } 

  // sumcheck it
  
  uint8_t sumCheck = 0;
  for (int i = 0; i < (bytesInLine - 1); i++)
    sumCheck += hexBuffer [i];
    
  // 2's complement
  sumCheck = ~sumCheck + 1;
  
  // check sumcheck
  if (sumCheck != hexBuffer [bytesInLine - 1])
    {
    printf("MSG_BAD_SUMCHECK\n\r");
    return true;
    }
  
  // length of data (eg. how much to write to memory)
  uint8_t len = hexBuffer [0];
  
  // the data length should be the number of bytes, less
  //   length / address (2) / transaction type / sumcheck
  if (len != (bytesInLine - 5))
    {
		printf("MSG_LINE_NOT_EXPECTED_LENGTH\n\r");
    return true;
    }
    
  // two bytes of address
  uint32_t addrH = hexBuffer [1];
  uint32_t addrL = hexBuffer [2];
  
  uint32_t addr = addrL | (addrH << 8);
  
  uint8_t recType = hexBuffer [3];

  switch (recType)
    {
    // stuff to be written to memory
    case hexDataRecord:
      lowestAddress  = min (lowestAddress, addr + extendedAddress);
      highestAddress = max (lowestAddress, addr + extendedAddress + len - 1);
      bytesWritten += len;
    
      switch (action)
        {
        case checkFile:  // nothing much to do, we do the checks anyway
          break;
          
        case verifyFlash:
          verifyData (spi, addr + extendedAddress, &hexBuffer [4], len);
		  //printf("\n\rverified in adress 0x%02X, len=%d, total=%d\n\r", addr + extendedAddress, len, bytesWritten);
          break;
        
        case writeToFlash:
          writeData (spi, addr + extendedAddress, &hexBuffer [4], len);
		  //printf("\n\rwritten to address 0x%02X, len=%d, total=%d\n\r", addr + extendedAddress, len, bytesWritten);
		  for (int j=0; j<len; j++)
		  {
			 //printf("0x%02X ", hexBuffer[4+j]);
		  }
		  
          break;      
        } // end of switch on action
      break;
  
    // end of data
    case hexEndOfFile:
      gotEndOfFile = true;
      break;
  
    // we are setting the high-order byte of the address
    case hexExtendedSegmentAddressRecord: 
      extendedAddress = ((uint32_t) hexBuffer [4]) << 12;
      break;
      
    // ignore these, who cares?
    case hexStartSegmentAddressRecord:
    case hexExtendedLinearAddressRecord:
    case hexStartLinearAddressRecord:
      break;
        
    default:  
      printf("MSG_UNKNOWN_RECORD_TYPE");
      return true;  
    }  // end of switch on recType
    
  return false;
  } // end of processLine

 
//------------------------------------------------------------------------------
// returns true if error, false if OK
bool readHexFile (spi_device_handle_t spi, const char * fName, const uint8_t action)
  {
  const int maxLine = 80;
  char buffer[maxLine];
  
  gotEndOfFile = false;
  extendedAddress = 0;
  errors = 0;
  lowestAddress = 0xFFFFFFFF;
  highestAddress = 0;
  bytesWritten = 0;
  
   ESP_LOGI(TAG, "Reading file %s", fName);

    // Open for reading hello.txt
    FILE* f = fopen(fName, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open GroundSt.hex");
        return true;
    }

  pagesize = 64;//currentSignature.pageSize;
  pagemask = (~(pagesize - 1));//>>1;
  oldPage = NO_PAGE;
  
  printf("PAGE MASK = 0x%02X PAGE SIZE = %d\n\r", pagemask, pagesize);

  switch (action)
    {
    case checkFile:
      break;
      
    case verifyFlash:
      break;
    
    case writeToFlash:

	  	eraseChip(spi);
        vTaskDelay(20 / portTICK_RATE_MS);
	    gpio_set_level(PIN_NUM_RST, 1);
		vTaskDelay(50 / portTICK_RATE_MS);
	    start_pmode(spi);	
	
	  clearPage(spi);  // clear temporary page
      break;      
    } // end of switch
 
  while (fgets(buffer, sizeof(buffer), f))//(sdin.getline (buffer, maxLine))
    {
    lineNumber++;
	int count = strlen(buffer)-1;
	for(int i = strlen(buffer)-1; i > 0; i--)
	{
	   if(buffer[i] == '\r' || buffer[i] == '\n')	
		   count--;
	   else
		   break;
	}
      
    // ignore empty lines
	printf("\n\rcount: %d\n\r", count);
    if (count > 1)
      {
      if (processLine (spi, buffer, action, count))
        {
        return true;  // error
        }
      }
    }    // end of while each line
    
  if (!gotEndOfFile)
    {
		printf("MSG_NO_END_OF_FILE_RECORD\n\r");
    return true;
    }

  switch (action)
    {
    case writeToFlash:
      // commit final page
      if (oldPage != NO_PAGE)
        commitPage (spi, oldPage);
      break;
      
    case verifyFlash:
       if (errors > 0)
          {
			  printf("MSG_VERIFICATION_ERROR %d\n\r", errors);
          return true;
          }  // end if
       break;
        
    case checkFile:
      break;
    }  // end of switch
  
  return false;
}  // end of readHexFile


bool target_poweron (spi_device_handle_t spi)
{
  printf("Starting Program Mode");
  start_pmode(spi);
  printf(" [OK]\n\r");
  return true;
}


 bool start_pmode (spi_device_handle_t spi) { 
	//printf("...spi_transaction");
	portDISABLE_INTERRUPTS(); 
	gpio_set_level(PIN_NUM_CLK, 0);

	//Initialize non-SPI GPIOs

	gpio_set_level(PIN_NUM_RST, 1);
	vTaskDelay(5 / portTICK_RATE_MS);

	//Reset the display
	gpio_set_level(PIN_NUM_RST, 0);
	vTaskDelay(50 / portTICK_RATE_MS);


	uint32_t check = spi_transaction(spi, 0xAC, 0x53, 0x00, 0x00);
	portENABLE_INTERRUPTS(); 
	if((check&0x0000FF00) == 0x5300)
	{
		printf("We are in programming mode!\n\r");
		return false;
	}
	else
	{
		printf("Switch to programming mode FAILED, %02X\n\r", check);
		return true;
	}
}

void updateFuses(spi_device_handle_t spi, uint8_t *fuses, uint8_t *fusemask)
{
	programFuses (spi, fuses);
	verifyFuses (spi, fuses, fusemask);
}

// returns true if OK, false on error
bool writeFlashContents (spi_device_handle_t spi, const char * name)
{
	errors = 0;
	
	// now fix up fuses so we can boot    
	//if (errors == 0)
		updateFuses (spi, wfuses, fuse_mask);

	//if (chooseInputFile ())
	//  return false;  

	// ensure back in programming mode  
	if (start_pmode(spi))
		return false;

	// now commit to flash
	if (readHexFile(spi, name, writeToFlash))
		return false;

	// verify
	if (readHexFile(spi, name, verifyFlash))
		return false;



	return errors == 0;
}  // end of writeFlashContents
  
void stopProgramming(spi_device_handle_t spi)
{
	spi_bus_remove_device(spi);
	spi_bus_free(HSPI_HOST);
	gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_INPUT);
	gpio_set_direction(PIN_NUM_CLK, GPIO_MODE_INPUT);
	gpio_set_direction(PIN_NUM_MISO, GPIO_MODE_INPUT);
}