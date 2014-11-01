/*
 * T_010.c
 * Firmware for rocket data bug "T-010"
 * ArtSimMagic Inc. dba AviRoc
 *
 * Created: 9/6/2014 4:20:24 PM
 *  Author: Brent W. York
 */ 

/*
Intended for use on an ATTiny4313
Fuses must be set as follows:
	lfuse: 0xef (stock is 0x64)
	hfuse: 0xdf (same as stock)
	efuse: 0xff (same as stock)
	lock:  0x3f (same as stock)

Do this by executing (on a factory-fresh IC):
	avrdude -c usbtiny -p attiny4313 -U lfuse:w:0xEF:m
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include "usbdrv.h"
#include "i2cmaster.h"

#define F_CPU 12000000L
#include <util/delay.h>


//-------------------------
//  GENERAL CONFIGURATION
//-------------------------
	// Data loop times in milliseconds
#define DATA_TICK              25			// 50 Hz data loop
#define HEARTBEAT_TICK       2000
#define HEARTBEAT_ON_TIME     100
#define HEARTBEAT_FULL_TIME   500
	// Altimeter configuration
		// Contents of CTRL_REG1
		// OSR = 8 (min time between samples = 18ms), altitude mode
const uint8_t MPL3115A2_CFG = 0x98;


//-----------------------------
//  I2C INTERFACE DEFINITIONS
//-----------------------------
const uint8_t MPL3115A2_DEVICE			= 0xC0;
const uint8_t MPL3115A2_REG_STATUS		= 0x00;
const uint8_t MPL3115A2_REG_OUT_P_MSB	= 0x01;
const uint8_t MPL3115A2_REG_OUT_P_CSB	= 0x02;
const uint8_t MPL3115A2_REG_OUT_P_LSB	= 0x03;
const uint8_t MPL3115A2_REG_OUT_T_MSB	= 0x04;
const uint8_t MPL3115A2_REG_OUT_T_LSB	= 0x05;
const uint8_t MPL3115A2_REG_WHOAMI		= 0x0C;
const uint8_t MPL3115A2_REG_PT_DATA_CFG	= 0x13;
const uint8_t MPL3115A2_REG_CTRL_REG1	= 0x26;
const uint8_t MPL3115A2_DEVICE_ID		= 0xC4;

const uint8_t MPU6050_DEVICE		= 0xD0;
const uint8_t MPU6050_REG_WHOAMI	= 0x75;
const uint8_t MPU6050_DEVICE_ID		= 0x68;


//-----------------------------
//  SPI INTERFACE DEFINITIONS
//-----------------------------
const uint16_t FLASH_JEDEC_ID = 0x0140;

const uint8_t FLASH_PAGE_PGM_CMD     = 0x02;
const uint8_t FLASH_READ_DATA_CMD    = 0x03;
const uint8_t FLASH_READ_SR1_CMD     = 0x05;
const uint8_t FLASH_WRITE_ENABLE_CMD = 0x06;
const uint8_t FLASH_SECTOR_ERASE_CMD = 0x20;
const uint8_t FLASH_JEDEC_ID_CMD     = 0x9F;
const uint8_t FLASH_CHIP_ERASE_CMD   = 0xC7;

#define FLASH_CS_SET  (PORTB &= ~_BV(PORTB4))		// set chip select (low is set)
#define FLASH_CS_CLR  (PORTB |= _BV(PORTB4))		// clear chip select (high is clear)


//-----------------------------
//  USB INTERFACE DEFINITIONS
//-----------------------------
	// USB messages from host
#define USB_STATUS				0
#define USB_PAGE_DUMP			1
#define USB_READ_ALTIMETER		2
#define USB_FLASH_STATUS		3
#define USB_CLEAR_MEMORY		4
#define USB_FLASH_WRITE_TEST	5
#define USB_FLASH_READ_TEST		6
#define USB_IS_FLASH_BUSY		7
#define USB_TEST				8
#define USB_DATA_SIZE			9
	// USB register configuration
	// Configures USI to 3-wire master mode with overflow interrupt
#define myUSICR  (_BV(USIWM0) | _BV(USICS1) | _BV(USICLK))
	// USB connection detection
#define USB_CONNECTED_PORT_IN	PINB
#define USB_CONNECTED_PIN_IN	PINB2
#define USB_CONNECTED			(USB_CONNECTED_PORT_IN & _BV(USB_CONNECTED_PIN_IN))


//--------------------
//  GLOBAL VARIABLES
//--------------------
	// General purpose data buffer
#define DATA_BUF_SZ 16
static uchar dataBuf[DATA_BUF_SZ];

	// Status byte
#define deviceStatus			(dataBuf[0])
#define DEV_GOOD_MPL3115		0
#define DEV_GOOD_FLASH			1
#define DEV_GOOD_MPU6050		2

	// Altimeter data
#define altitude_ptr			(dataBuf + 1)		// 3 bytes
#define temperature_ptr			(dataBuf + 4)		// 2 bytes

	// Flash memory
static struct {
	uint16_t size;			// size in 256-byte pages
	uint16_t startPage;		// starting write page
	uint16_t page;			// current write page indicator
	uint8_t  byteCtr;		// current write byte counter within page
	uint8_t  full;			// memory full flag
} flash;



//---------------------------
//  I2C INTERFACE FUNCTIONS
//---------------------------

uint8_t writeI2CRegister(uint8_t device, uint8_t reg, uint8_t val)
{
	if (i2c_start(device | I2C_WRITE) != 0) return 0;
	i2c_write(reg);
	i2c_write(val);
	i2c_stop();
	return 1;
}


uint8_t readI2CRegister(uint8_t device, uint8_t reg)
{
	uint8_t val;
	
	i2c_start_wait(device | I2C_WRITE);
	i2c_write(reg);
	i2c_rep_start(device | I2C_READ);
	val = i2c_readNak();
	i2c_stop();
	
	return val;
}


uint8_t writeMPL3115Register(uint8_t reg, uint8_t val)
{
	if (i2c_start(MPL3115A2_DEVICE | I2C_WRITE) != 0) return 0;
	i2c_write(reg);
	i2c_write(val);
	i2c_stop();
	return 1;
}


uint8_t readMPL3115Register(uint8_t reg)
{
	uint8_t val;
	
	i2c_start_wait(MPL3115A2_DEVICE | I2C_WRITE);
	i2c_write(reg);
	i2c_rep_start(MPL3115A2_DEVICE | I2C_READ);
	//i2c_readAck(); // dummy read
	val = i2c_readNak();
	i2c_stop();
	
	return val;
}


uint8_t initMPL3115(void)
// Initialize the MPL3115A2 pressure sensor / altimeter, return 0 if bad comms
{
// Check that expected device is present
	if (readMPL3115Register(MPL3115A2_REG_WHOAMI) != MPL3115A2_DEVICE_ID) return 0;

// Set altimeter configuration
	if (!writeMPL3115Register(MPL3115A2_REG_CTRL_REG1, MPL3115A2_CFG)) return 0;

// Enable data flags in PT_DATA_CFG
	if (!writeMPL3115Register(MPL3115A2_REG_PT_DATA_CFG, 0x07)) return 0;

// Start data collection
//	if (!writeMPL3115Register(MPL3115A2_REG_CTRL_REG1, MPL3115A2_CFG | 1)) return 0;
	
	return 1;	// device good
}


void startMPL3115OneShot(void)
// Starts a one-shot data acquisition
{
	readMPL3115Register(MPL3115A2_REG_CTRL_REG1);							// auto-clear one-shot bit
	writeMPL3115Register(MPL3115A2_REG_CTRL_REG1, MPL3115A2_CFG | 0x02);	// set one-shot bit
}


void waitForMPL3115NewData(void)
// Check for new data ready
{
// TODO: Add a timeout?
	uint8_t status;
	do {
		status = readMPL3115Register(MPL3115A2_REG_STATUS);
	} while ((status & 0x08) == 0);
}


void readMPL3115(void)
// Read altitude and temperature data from MPL3115A2 into data buffer
//  Altitude:	 24 bits = BB.B  (8-bit fractional part) (meters MSL)
//  Temperature: 12 bits =  B.B  (4-bit fractional part) (degrees C)
{
	uint8_t *p = altitude_ptr;
	
	waitForMPL3115NewData();

#if 0

// Read altitude
	*(p++) = readMPL3115Register(MPL3115A2_REG_OUT_P_MSB);
	*(p++) = readMPL3115Register(MPL3115A2_REG_OUT_P_CSB);
	*(p++) = readMPL3115Register(MPL3115A2_REG_OUT_P_LSB);
	
// Read temperature
	*(p++) = readMPL3115Register(MPL3115A2_REG_OUT_T_MSB);
	*p = readMPL3115Register(MPL3115A2_REG_OUT_T_LSB);
	
#else

	uchar i;
	
	i2c_start_wait(MPL3115A2_DEVICE | I2C_WRITE);
	i2c_write(MPL3115A2_REG_OUT_P_MSB);
	i2c_rep_start(MPL3115A2_DEVICE | I2C_READ);
	for (i=0; i<4; i++) *(p++) = i2c_readAck();
	*p = i2c_readNak();
	i2c_stop();

#endif
}


uint8_t initMPU6050(void)
// Initialize the MPU-6050 accelerometer, return 0 if bad comms
{
	if (readI2CRegister(MPU6050_DEVICE, MPU6050_REG_WHOAMI) != MPU6050_DEVICE_ID) return 0;
	return 1;
}



//---------------------------
//  SPI INTERFACE FUNCTIONS
//---------------------------

uint8_t spiByte(uint8_t data)
// SPI write / read one byte
{
	USIDR = data;
	USISR = _BV(USIOIF);				// clear flag and set for 8 ticks
	
	while ((USISR & _BV(USIOIF)) == 0) {
		//_delay_us(500);
		USICR = myUSICR | _BV(USITC);	// clock tick
	}
	return USIDR;
}


uint8_t initFlash(void)
// Initialize the S25FL1--K Flash chip, return 0 if bad comms
{
	uint16_t id;
	uint8_t sz;

	// Verify device ID
	FLASH_CS_SET;
	spiByte(FLASH_JEDEC_ID_CMD);
	id = (spiByte(0) << 8) | spiByte(0);	if (id != FLASH_JEDEC_ID) {		FLASH_CS_CLR;
		return 0;	}	
	// Read flash size
	sz = spiByte(0) - 0x14;
	flash.size = sz * 2;
	if (flash.size == 6) flash.size = 8;
	flash.size <<= 12; // * 4096

	FLASH_CS_CLR;	
	return 1;	// device good
}


uint8_t isFlashBusy(void)
// Check status register to see if device is busy, returns 1 if busy or 0 if ready
{
	uint8_t ret;
	
	// Read status register 1
	FLASH_CS_SET;
	spiByte(FLASH_READ_SR1_CMD);
	ret = spiByte(0) & 1;
	FLASH_CS_CLR;
	
	// Return bit 0
	return ret;
}


void waitForFlashReady(void)
// Waits until device status indicates ready to accept commands
{
	// Read status register 1
	FLASH_CS_SET;
	spiByte(FLASH_READ_SR1_CMD);
	while (spiByte(0) & 1)
		;
	FLASH_CS_CLR;
}


void enableFlashWrites(void)
// Enables writing to the flash; must be called before each new write operation
{
	FLASH_CS_SET;
	spiByte(FLASH_WRITE_ENABLE_CMD);
	FLASH_CS_CLR;
	_delay_us(20); // TODO: check length of delay
}


void eraseFlashSector(uint32_t addr)
// Erases the 4-KB sector containing the given address
{
	waitForFlashReady();
	enableFlashWrites();
	FLASH_CS_SET;
	spiByte(FLASH_SECTOR_ERASE_CMD);
	spiByte((addr >> 16) & 0xFF);
	spiByte((addr >> 8) & 0xFF);
	spiByte(addr & 0xFF);
	FLASH_CS_CLR;
}


void eraseFlash(void)
// Erases the entire flash chip (takes 30-256 seconds to complete!)
{
	waitForFlashReady();
	enableFlashWrites();
	FLASH_CS_SET;
	spiByte(FLASH_CHIP_ERASE_CMD);
	FLASH_CS_CLR;
}


void writeFlashPageNumber(uint16_t page)
// Updates page number
{
#if 0
// (stored at start of page 0 so it survives power cycles)
	eraseFlashSector(0);
	waitForFlashReady();
	enableFlashWrites();
	FLASH_CS_SET;
	spiByte(FLASH_PAGE_PGM_CMD);
	spiByte(0);	// page 0 (MSB)
	spiByte(0);	// page 0 (LSB)
	spiByte(0); // byte 0
	spiByte(page >> 8);
	spiByte(page & 0xFF);
	FLASH_CS_CLR;
#else
// (stored in first two bytes of EEPROM so it survives power cycles)
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set Programming mode */
	EECR = (0<<EEPM1)|(0<<EEPM0);
	/* Set up address and data registers */
	EEAR = 0;
	EEDR = page & 0xFF;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);

	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set Programming mode */
	EECR = (0<<EEPM1)|(0<<EEPM0);
	/* Set up address and data registers */
	EEAR = 1;
	EEDR = page >> 8;
	/* Write logical one to EEMPE */
	EECR |= (1<<EEMPE);
	/* Start eeprom write by setting EEPE */
	EECR |= (1<<EEPE);

#endif
}


void startFlashWritePage(uint16_t page)
// Opens a flash memory page for writing. Called automatically for sequential writes.
{
	writeFlashPageNumber(page);

// Open the indicated page for writing
	waitForFlashReady();
	enableFlashWrites();
	FLASH_CS_SET;
	spiByte(FLASH_PAGE_PGM_CMD);
	spiByte(page >> 8);
	spiByte(page & 0xFF);
	spiByte(0);  // start at byte 0
}


void startFlashWrite(uint16_t page)
// Begins a flash memory sequential write.
{
	flash.page = page;
	flash.startPage = page;
	flash.byteCtr = 0;
	startFlashWritePage(page);
}


uint8_t writeFlashByte(uint8_t b)
// Writes bytes sequentially to the flash memory.
// Must have called startFlashWrite() before calling this function.
// Returns 1 if we just filled the last memory page; 0 if all OK.
{
	if (!flash.full) {
		spiByte(b);
		if (++flash.byteCtr == 0) {
			FLASH_CS_CLR;
			flash.page++;
//			if (flash.page > flash.size) flash.page = 0;
//			if (flash.page == flash.startPage) flash.full = 1;
			if (flash.page > flash.size) flash.full = 1;  // no wrap-around for now...
			else startFlashWritePage(flash.page);
		}
	}
	return flash.full;
}

/*
void endFlashWrite(void)
// Ends a flash memory sequential write
{
	FLASH_CS_CLR;
}
*/

void startFlashReadPage(uint16_t page)
// Starts a sequential read at the beginning of a page
{
	FLASH_CS_CLR;
	waitForFlashReady();
	FLASH_CS_SET;
	spiByte(FLASH_READ_DATA_CMD);
	spiByte(page >> 8);		// page MSB
	spiByte(page & 0xFF);	// page LSB
	spiByte(0);				// byte 0
}


void readLastFlashPage(void)
// Reads last page used
{
#if 0
// ...from the flash memory (survives power cycle)
	flash.full = 0;
	waitForFlashReady();
	FLASH_CS_SET;
	spiByte(FLASH_READ_DATA_CMD);
	spiByte(0);	// page 0 (MSB)
	spiByte(0);	// page 0 (LSB)
	spiByte(0); // byte 0
	flash.page = (spiByte(0) << 8) + spiByte(0);
	FLASH_CS_CLR;
#else
// ...from the EEPROM (survives power cycle)
	flash.full = 0;
	
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEAR = 0;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	flash.page = EEDR;
	
	/* Wait for completion of previous write */
	while(EECR & (1<<EEPE));
	/* Set up address register */
	EEAR = 1;
	/* Start eeprom read by writing EERE */
	EECR |= (1<<EERE);
	/* Return data from data register */
	flash.page |= ((uint16_t)EEDR) << 8;
	
#endif

	if (flash.page == 0xFFFF) flash.page = 0;  // uninitialized case
	if (flash.page > flash.size) flash.full = 1;
}


uint8_t resetDataPtr(void)
// "Clears" memory by removing any saved data start pointer
{
	flash.page = 0;
	writeFlashPageNumber(flash.page);
	flash.full = 0;
	flash.byteCtr = 0;
	eraseFlash();
	return 1;	// returns non-zero on success...
}



//---------------------------
//  USB INTERFACE FUNCTIONS
//---------------------------

USB_PUBLIC usbMsgLen_t usbFunctionSetup(uchar data[8])
// Responds to USB messages
{
	uchar i;
	usbRequest_t *rq = (usbRequest_t *)data;
	
	// Respond to command in bRequest field
	switch(rq->bRequest) {

		case USB_CLEAR_MEMORY:
			if (!resetDataPtr()) return 0;
		case USB_STATUS:		// intentional fall-through
			usbMsgPtr = (usbMsgPtr_t)(&deviceStatus);
			return 1;

		case USB_TEST:
		// Currently just dumps all data currently in the general
		//  purpose buffer to the host
			usbMsgPtr = (usbMsgPtr_t)dataBuf;
			return DATA_BUF_SZ;

		case USB_READ_ALTIMETER:
			startMPL3115OneShot();
			readMPL3115();
			usbMsgPtr = (usbMsgPtr_t)(altitude_ptr);
			return 5;
			
		case USB_FLASH_STATUS:
			usbMsgPtr = (usbMsgPtr_t)(&flash);
			return sizeof(flash);
			
		case USB_FLASH_WRITE_TEST:
		// Write test data at byte 64 on page 0
			FLASH_CS_CLR;
			eraseFlashSector(0);
			waitForFlashReady();
			enableFlashWrites();
			FLASH_CS_SET;
			spiByte(FLASH_PAGE_PGM_CMD);
			spiByte(0);	// page 0 (MSB)
			spiByte(0);	// page 0 (LSB)
			spiByte(64); // byte 64
			spiByte(rq->wValue.bytes[0]);
			spiByte(rq->wValue.bytes[1]);
			spiByte(rq->wIndex.bytes[0]);
			spiByte(rq->wIndex.bytes[1]);
			FLASH_CS_CLR;
			return 0;
		
		case USB_FLASH_READ_TEST:
		// Read test data from byte 64 on page 0 and send to host
			FLASH_CS_CLR;
			waitForFlashReady();
			FLASH_CS_SET;
			spiByte(FLASH_READ_DATA_CMD);
			spiByte(0);	// page 0 (MSB)
			spiByte(0);	// page 0 (LSB)
			spiByte(64); // byte 64
			for (i=0; i<4; i++)
				dataBuf[i+10] = spiByte(0);
			FLASH_CS_CLR;
			usbMsgPtr = (usbMsgPtr_t)(dataBuf+10);
			return 4;
			
		case USB_IS_FLASH_BUSY:
		// Checks whether flash RAM is busy (typically because it's doing a chip erase)
			dataBuf[15] = isFlashBusy();
			usbMsgPtr = (usbMsgPtr_t)(dataBuf+15);
			return 1;
			
		case USB_DATA_SIZE:
		// Returns size of data stored in flash RAM in 256-byte pages
			usbMsgPtr = (usbMsgPtr_t)(&flash.page);
			return sizeof(flash.page);
			
		case USB_PAGE_DUMP:
		// Sends specified page of data to the host (page number in wIndex)
			startFlashReadPage(rq->wIndex.word);
			flash.byteCtr = 0;
			return USB_NO_MSG;
 	}

	return 0; // unhandled message
}


USB_PUBLIC uchar usbFunctionRead(uchar *data, uchar len)
{
	uchar i;

	for (i=0; i<len; i++) {
		data[i] = spiByte(0);
		if (++flash.byteCtr == 0) {
			i++;
			FLASH_CS_CLR;
			break;
		}
	}
	
	return i;
}



//----------------
//  MAIN PROGRAM
//----------------

int main(void)
{
	uchar i;
	uint16_t pacifier = 0;
	uchar usbConnected, usbConnectedLast = 0;
	uchar *p;
	uint16_t PACIFIER_OFF_TGT = HEARTBEAT_TICK / DATA_TICK;
	uint16_t PACIFIER_ON_TGT = PACIFIER_OFF_TGT - (HEARTBEAT_ON_TIME / DATA_TICK);
	uint16_t PACIFIER_FULL_TGT = HEARTBEAT_FULL_TIME / DATA_TICK;
	
// Enable 1 sec watchdog timer
	wdt_enable(WDTO_1S);

// Set up pins
								// PA0: Crystal
								// PA1: Crystal
								// PA2: !RESET
	// input					// PB0: Altimeter interrupt 1
	// input					// PB1: Altimeter interrupt 2
	// input					// PB2: USB connection present flag
	PORTB &= ~_BV(PORTB2);		    // no pull-up
								// PB3: reserved for MPU interrupt
	DDRB |= _BV(DDB4);			// PB4: !CS for SPI
	PORTB |= _BV(PORTB4);			// high (off) to start
	// input					// PB5: DI for SPI (shared with ISP)
	PORTB |= _BV(PORTB5);			// pull-up on DI
	DDRB |= _BV(DDB6);			// PB6: DO for SPI (shared with ISP)
	DDRB |= _BV(DDB7);			// PB7: SCK for SPI (shared with ISP)
	DDRD |= _BV(DDD0);			// PD0: SCL for I2C
	DDRD |= _BV(DDD1);			// PD1: SDA for I2C
	// V-USB handles this pin	// PD2: USB+
	// V-USB handles this pin	// PD3: USB-
								// PD4: spare
								// PD5: spare
	DDRD |= _BV(DDD6);			// PD6: Status LED (output)

// Initialize software I2C
	i2c_init();

// Enable SPI mode for USI (hardware SPI)
	USICR = myUSICR;
	
// Initialize all bus devices
	deviceStatus = 0;
	deviceStatus |= initMPL3115() << DEV_GOOD_MPL3115;
	deviceStatus |= initFlash() << DEV_GOOD_FLASH;
	deviceStatus |= initMPU6050() << DEV_GOOD_MPU6050;
	deviceStatus |= flash.size >> 8;		// store size of flash RAM (in MB) to top 3 bits of status byte

// Initialize USB
	// TODO: Check whether this is working as expected for start-up when not connected to USB
	usbInit();
	usbDeviceDisconnect(); // enforce re-enumeration
	for (i=0; i<10; i++) { // wait 500 ms
		wdt_reset();
		_delay_ms(50);
	}
	usbDeviceConnect();
	sei(); // Enable interrupts after re-enumeration

// Initialize flash memory
	readLastFlashPage();
	if (flash.page != 0) flash.page++;  // start at page following last page used (unless memory is clear)
	startFlashWrite(flash.page);
	
// Take initial altimeter data point
	startMPL3115OneShot();

// Main loop
	while (1) {
		// Reset watchdog
		wdt_reset();
		
		// If connected to USB, poll for messages (and keep status light on continuously)
		usbConnected = USB_CONNECTED;
		if (usbConnected) {
			// if (!usbConnectedLast) re-enumerate?
			PORTD |= _BV(PORTD6); // turn status LED on
			usbPoll();
		}
		
		// If NOT connected to USB, collect data (and flash status light as heartbeat)
		else {
			if (usbConnectedLast) {
				PORTD &= ~_BV(PORTD6); // turn status LED off immediately if just removed
				pacifier = 0;
				startMPL3115OneShot();
			}
			
			// Read altimeter
			readMPL3115();
			startMPL3115OneShot();
			
			// Write data to flash (altitude and temperature)
			// TODO: Consider compacting into only 4 bytes (because these data are 20-bit and 12-bit, respectively)
			p = altitude_ptr;
			for (i=0; i<5; i++) writeFlashByte(*(p++));

			_delay_ms(DATA_TICK);  // should instead wait for a clock tick, not delay a specified amount!
			
			// Heartbeat
			pacifier++;
			if (flash.full) {
				// Flash quickly if the memory is filled
				if (pacifier > PACIFIER_FULL_TGT) {
					PIND |= _BV(PORTD6); // toggle status LED
					pacifier = 0;
				}
			}
			else if (pacifier > PACIFIER_ON_TGT) {
				// Flash slowly for normal operation
				if (pacifier > PACIFIER_OFF_TGT) {
					PORTD &= ~_BV(PORTD6); // turn status LED off
					pacifier = 0;
				}
				else PORTD |= _BV(PORTD6); // turn status LED on
			}
		}
		
		usbConnectedLast = usbConnected;
	}
	
	return 0;  // never reached
}
