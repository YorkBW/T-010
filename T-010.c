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
	// Define USING_MPL3115 if the newer chip is present;
	//  else MPL115 is assumed
#define USING_MPL3115	1
	// Data loop times in milliseconds
#define DATA_TICK              50
#define HEARTBEAT_TICK       2000
#define HEARTBEAT_ON_TIME     100
#define HEARTBEAT_FULL_TIME   500


//-----------------------------
//  I2C INTERFACE DEFINITIONS
//-----------------------------
const uint8_t MPLx115A2_DEVICE        = 0xC0;
const uint8_t MPL115A2_READPRESMSB    = 0x00;
const uint8_t MPL115A2_READPRESLSB    = 0x01;
const uint8_t MPL115A2_READTEMPMSB    = 0x02;
const uint8_t MPL115A2_READTEMPLSB    = 0x03;
const uint8_t MPL115A2_READCOEFFIRST  = 0x04;
const uint8_t MPL115A2_READCOEFLAST   = 0x0B;
const uint8_t MPL115A2_STARTCONV      = 0x12;

const uint8_t MPU6050_DEVICE		  = 0xD0;
const uint8_t MPU6050_WHOAMI		  = 0x75;


//-----------------------------
//  SPI INTERFACE DEFINITIONS
//-----------------------------
const uint16_t FLASH_JEDEC_ID = 0x0140;

const uint8_t FLASH_PAGE_PGM_CMD     = 0x02;
const uint8_t FLASH_READ_SR1_CMD     = 0x05;
const uint8_t FLASH_WRITE_ENABLE_CMD = 0x06;
const uint8_t FLASH_JEDEC_ID_CMD     = 0x9f;

#define FLASH_CS_SET  (PORTB &= !_BV(PORTB4))		// set chip select (low is set)
#define FLASH_CS_CLR  (PORTB |= _BV(PORTB4))		// clear chip select (high is clear)


//-----------------------------
//  USB INTERFACE DEFINITIONS
//-----------------------------
	// USB messages from host
#define USB_STATUS			0
#define USB_DATA_DUMP		1
#define USB_ALTITUDE		2
#define USB_TEMPERATURE		3
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
#define DEV_GOOD_MPLX115		0
#define DEV_GOOD_FLASH			1
#define DEV_GOOD_MPU6050		2
uchar deviceStatus;

	// MPL115A2 coefficients
#if !defined(USING_MPL3115)
int16_t a0, b1, b2, c12;
#endif

	// Flash memory
uint16_t flashSize;			// size in 256-byte pages
uint16_t flashStartPage;	// starting write page
uint16_t flashPage;			// current write page indicator
uint8_t flashByteCtr;		// current write byte counter within page


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


uint8_t writeMPLx115Register(uint8_t reg, uint8_t val)
{
	if (i2c_start(MPLx115A2_DEVICE | I2C_WRITE) != 0) return 0;
	i2c_write(reg);
	i2c_write(val);
	i2c_stop();
	return 1;
}


uint8_t readMPLx115Register(uint8_t reg)
{
	uint8_t val;
	
	i2c_start_wait(MPLx115A2_DEVICE | I2C_WRITE);
	i2c_write(reg);
	i2c_rep_start(MPLx115A2_DEVICE | I2C_READ);
	//i2c_readAck(); // dummy read
	val = i2c_readNak();
	i2c_stop();
	
	return val;
}


uint8_t initMPLx115(void)
// Initialize the MPLx115A2 pressure sensor / altimeter, return 0 if bad comms
{
#ifdef USING_MPL3115

// Set to altimeter with an OSR = 128
	if (!writeMPLx115Register(0x26, 0xB8)) return 0;

// Set user offset altitude (TODO: make a real offset, not a test!)
	//if (!writeMPLx115Register(0x2D, 35)) return 0;

// Enable data flags in PT_DATA_CFG
	if (!writeMPLx115Register(0x13, 0x07)) return 0;

// Start data collection
	if (!writeMPLx115Register(0x26, 0xB9)) return 0;
	
	// TEMP device check!
	dataBuf[1] = readMPLx115Register(0x0C);
	dataBuf[2] = 0xFF;

#else

	// Init comms
	if (i2c_start(MPLx115A2_DEVICE | I2C_WRITE) != 0) return 0;
	
	// Read coefficients
	i2c_write(MPL115A2_READCOEFFIRST);
	i2c_rep_start(MPLx115A2_DEVICE | I2C_READ);
	for (uchar coef=MPL115A2_READCOEFFIRST; coef<=MPL115A2_READCOEFLAST; coef++) {
		dataBuf[coef - MPL115A2_READCOEFFIRST] = i2c_readAck();
	}
	
	i2c_stop();
	
	a0  = (int16_t)((dataBuf[0] << 8) | dataBuf[1]);	// 3 fractional bits
	b1  = (int16_t)((dataBuf[2] << 8) | dataBuf[3]);	// 13 fractional bits
	b2  = (int16_t)((dataBuf[4] << 8) | dataBuf[5]);	// 14 fractional bits
	c12 = (int16_t)((dataBuf[6] << 8) | dataBuf[7]);    // 9 dec pt zero pad

#endif

	return 1;	// device good
}


void startMPLx115Read(void)
{
#ifdef USING_MPL3115
// Check for data ready
// TODO: Add a timeout!
	uint8_t status;
	do {
		status = readMPLx115Register(0x00);
	} while ((status & 0x08) == 0);
#else
// Start ADC conversion on MPL115A2 (needs up to 3ms to complete)
	i2c_start_wait(MPLx115A2_DEVICE | I2C_WRITE);
	i2c_write(MPL115A2_STARTCONV);
	i2c_write(0);
	i2c_stop();
#endif
}


int32_t readMPLx115Altitude(void)
// Read data from MPLx115A2 and return altitude with 5-bit fractional part... (was pressure in kPa (with 4-bit fractional part))
{
#ifdef USING_MPL3115
	uint8_t OUT_P_MSB, OUT_P_CSB, OUT_P_LSB;
	int16_t res;
	
	// Read OUT_P and OUT_T
	OUT_P_MSB = readMPLx115Register(0x01);
	OUT_P_CSB = readMPLx115Register(0x02);
	OUT_P_LSB = readMPLx115Register(0x03);
	//OUT_T_MSB = readMPLx115Register(0x04);
	//OUT_T_LSB = readMPLx115Register(0x05);
	dataBuf[3] = OUT_P_MSB;
	dataBuf[4] = OUT_P_CSB;
	dataBuf[5] = OUT_P_LSB;
	
	dataBuf[10] = readMPLx115Register(0x04);
	dataBuf[11] = readMPLx115Register(0x05);
	
	dataBuf[7] = OUT_P_MSB;
	dataBuf[8] = OUT_P_CSB;
	dataBuf[9] = OUT_P_LSB;
	if (OUT_P_MSB & 0x80) {
		dataBuf[6] = 1;
		if (dataBuf[9]-- == 0) {
			if (dataBuf[8]-- == 0) {
				dataBuf[7]--;
			}
		}
		dataBuf[7] = ~dataBuf[7];
		dataBuf[8] = ~dataBuf[8];
		dataBuf[9] = ~dataBuf[9];
	}
	else {
		dataBuf[6] = 0;
	}
	
	
	res = (((uint16_t)OUT_P_MSB) << 13) & (((uint16_t)OUT_P_CSB) << 5) & (OUT_P_LSB >> 3);	// TODO: Check for negatives...this is 2s-complement data
	return res;
	
#else

	uint16_t Padc, Tadc;
	int32_t c12x2, a1, a1x1, y1, a2x2, PComp;
	
	i2c_start_wait(MPLx115A2_DEVICE | I2C_WRITE);
	i2c_write(0);
	i2c_rep_start(MPLx115A2_DEVICE | I2C_READ);
	Padc = (i2c_readAck() << 2) + (i2c_readAck() >> 6);
	Tadc = (i2c_readAck() << 2) + (i2c_readNak() >> 6);
	i2c_stop();

	c12x2 = (((int32_t)c12) * Tadc) >> 11;	// c12x2 = c12 * Tadc
	a1    = (int32_t)b1 + c12x2;			// a1    = b1  + c12x2
	a1x1  = a1 * Padc;						// a1x1  = a1  * Padc
	y1    = (((int32_t)a0) << 10) + a1x1;	// y1    = a0  + a1x1
	a2x2  = (((int32_t)b2) * Tadc) >> 1;	// a2x2  = b2  * Tadc
	PComp = (y1 + a2x2) >> 9;				// PComp = y1  + a2x2

	return (int16_t)((((int32_t)PComp) * 1041) >> 14) + 800;		// TODO: Convert to altitude!

#endif
}


int16_t readMPLx115Temperature(void)
// Read data from MPLx115A2 and return temperature in DegC
{
#ifdef USING_MPL3115

	uint8_t OUT_T_MSB, OUT_T_LSB;
	int16_t res;
	
	// Read OUT_P and OUT_T
	OUT_T_MSB = readMPLx115Register(0x04);
	OUT_T_LSB = readMPLx115Register(0x05);
	dataBuf[3] = OUT_P_MSB;
	dataBuf[4] = OUT_P_CSB;
	dataBuf[5] = OUT_P_LSB;
	
	dataBuf[10] = readMPLx115Register(0x04);
	dataBuf[11] = readMPLx115Register(0x05);
	
	dataBuf[7] = OUT_P_MSB;
	dataBuf[8] = OUT_P_CSB;
	dataBuf[9] = OUT_P_LSB;
	if (OUT_P_MSB & 0x80) {
		dataBuf[6] = 1;
		if (dataBuf[9]-- == 0) {
			if (dataBuf[8]-- == 0) {
				dataBuf[7]--;
			}
		}
		dataBuf[7] = ~dataBuf[7];
		dataBuf[8] = ~dataBuf[8];
		dataBuf[9] = ~dataBuf[9];
	}
	else {
		dataBuf[6] = 0;
	}
	
	
	res = (((uint16_t)OUT_P_MSB) << 13) & (((uint16_t)OUT_P_CSB) << 5) & (OUT_P_LSB >> 3);	// TODO: Check for negatives...this is 2s-complement data
	return res;
	
#else

	uint16_t Padc, Tadc;
	int32_t c12x2, a1, a1x1, y1, a2x2, PComp;
	
	i2c_start_wait(MPLx115A2_DEVICE | I2C_WRITE);
	i2c_write(0);
	i2c_rep_start(MPLx115A2_DEVICE | I2C_READ);
	Padc = (i2c_readAck() << 2) + (i2c_readAck() >> 6);
	Tadc = (i2c_readAck() << 2) + (i2c_readNak() >> 6);
	i2c_stop();

	c12x2 = (((int32_t)c12) * Tadc) >> 11;	// c12x2 = c12 * Tadc
	a1    = (int32_t)b1 + c12x2;			// a1    = b1  + c12x2
	a1x1  = a1 * Padc;						// a1x1  = a1  * Padc
	y1    = (((int32_t)a0) << 10) + a1x1;	// y1    = a0  + a1x1
	a2x2  = (((int32_t)b2) * Tadc) >> 1;	// a2x2  = b2  * Tadc
	PComp = (y1 + a2x2) >> 9;				// PComp = y1  + a2x2

	return (int16_t)((((int32_t)PComp) * 1041) >> 14) + 800;		// TODO: Convert to altitude!

#endif
}


uint8_t initMPU6050(void)
// Initialize the MPU-6050 accelerometer, return 0 if bad comms
{
	uint8_t c;
	
	c = readI2CRegister(MPU6050_DEVICE, MPU6050_WHOAMI);
	if (c != 0x68) return 0;
	
	return 1;
}



//---------------------------
//  SPI INTERFACE FUNCTIONS
//---------------------------

uint8_t spiByte(uint8_t data)
// SPI write / read one byte
{
	USIDR = data;
	USISR = _BV(USIOIF) /*| 0x08*/; // clear flag and set for 8 ticks
	
	while ((USISR & _BV(USIOIF)) == 0) {
		//_delay_us(500);
		USICR = myUSICR | _BV(USITC); // clock tick
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
	id = (spiByte(0) << 8) | spiByte(0);	if (id != FLASH_JEDEC_ID) return 0;	
	// Read flash size
	sz = spiByte(0) - 0x14;
	deviceStatus = (deviceStatus & 0x3F) | (sz << 6);
	flashSize = sz * 2;
	if (flashSize == 6) flashSize = 8;
	flashSize *= 4096;

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
	ret = spiByte(0)&1;
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
	while (spiByte(0)&1)
		;
	FLASH_CS_CLR;
}


void startFlashWritePage(uint16_t page)
// Opens a flash memory page for writing. Called automatically for sequential writes.
{
	FLASH_CS_SET;
	spiByte(FLASH_WRITE_ENABLE_CMD);
	FLASH_CS_CLR;
	_delay_us(20); // TODO: check length of delay
	
	FLASH_CS_SET;
	spiByte(FLASH_PAGE_PGM_CMD);
	spiByte(page >> 8);
	spiByte(page & 0xFF);
	spiByte(0);
}


void startFlashWrite(uint16_t page)
// Begins a flash memory sequential write.
{
	flashPage = page;
	flashStartPage = page;
	flashByteCtr = 0;
	startFlashWritePage(page);
}


uint8_t writeFlashByte(uint8_t b)
// Writes bytes sequentially to the flash memory.
// Must have called startFlashWrite() before calling this function.
// Returns 1 if we just filled the last memory page; 0 if all OK.
{
	spiByte(b);
	if (flashByteCtr++ == 0) {
		FLASH_CS_CLR;
		waitForFlashReady();
		flashPage++;
		if (flashPage > flashSize) flashPage = 0;
		if (flashPage == flashStartPage) return 1;
		startFlashWritePage(flashPage);
	}
	return 0;
}


void endFlashWrite(void)
// Ends a flash memory sequential write.
{
	FLASH_CS_CLR;
}




//---------------------------
//  USB INTERFACE FUNCTIONS
//---------------------------

USB_PUBLIC uchar usbFunctionSetup(uchar data[8])
// Responds to USB messages
{
	//int temp;
	//uint16_t utemp;
	//int16_t altitude;
	
	usbRequest_t *rq = (usbRequest_t *)data;
	
	// Respond to command in bRequest field
	switch(rq->bRequest) {

		case USB_STATUS:
			usbMsgPtr = (usbMsgPtr_t)(&deviceStatus);
			return 1;

		case USB_DATA_DUMP: // send data to PC
			usbMsgPtr = (usbMsgPtr_t)dataBuf;
			return DATA_BUF_SZ;

		case USB_ALTITUDE:
			startMPLx115Read();
			#if !defined(USING_MPL3115)
			_delay_ms(3);
			#endif
			/* altitude = */readMPLx115Altitude();
			//usbMsgPtr = (usbMsgPtr_t)(&altitude);
			//return sizeof(altitude);
			usbMsgPtr = (usbMsgPtr_t)(dataBuf + 3);
			return 3;

		case USB_TEMPERATURE:
			startMPLx115Read();
			#if !defined(USING_MPL3115)
			_delay_ms(3);
			#endif
			/* altitude = */readMPLx115Temperature();
			usbMsgPtr = (usbMsgPtr_t)(dataBuf + 3);
			return 3;

/*
		case USB_DATA_WRITE: // modify reply buffer
			replyBuf[7] = rq->wValue.bytes[0];
			replyBuf[8] = rq->wValue.bytes[1];
			replyBuf[9] = rq->wIndex.bytes[0];
			replyBuf[10] = rq->wIndex.bytes[1];
			return 0;
*/
 	}

	return 0; // unhandled message
}



//----------------
//  MAIN PROGRAM
//----------------

int main(void)
{
	uchar i, memoryFull;
	uint16_t pacifier = 0;
	uchar usbConnected, usbConnectedLast = 0;
	uint16_t PACIFIER_OFF_TGT = HEARTBEAT_TICK / DATA_TICK;
	uint16_t PACIFIER_ON_TGT = PACIFIER_OFF_TGT - (HEARTBEAT_ON_TIME / DATA_TICK);
	uint16_t PACIFIER_FULL_TGT = HEARTBEAT_FULL_TIME / DATA_TICK;
	int16_t altitude;
	
#define CHECK_WRITE(b) { if (!memoryFull) memoryFull = writeFlashByte(b); }

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
	deviceStatus |= initMPLx115() << DEV_GOOD_MPLX115;
	deviceStatus |= initFlash() << DEV_GOOD_FLASH;
	deviceStatus |= initMPU6050() << DEV_GOOD_MPU6050;
	dataBuf[0] = deviceStatus;

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
	
	memoryFull = 0;
	startFlashWrite(0);		// temp -- starts at beginning of flash whenever turned on
							// TODO: come up with something more intelligent...

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
			if (usbConnectedLast) PORTD &= ~_BV(PORTD6); // turn status LED off immediately if just removed
			
			// Read altimeter
			startMPLx115Read();
			_delay_ms(3);  // maximum ADC conversion time
			altitude = readMPLx115Altitude();
			
			// Write data to flash
			CHECK_WRITE(((uint16_t)altitude) >> 8);
			CHECK_WRITE(((uint16_t)altitude) & 0xFF);

			_delay_ms(DATA_TICK);  // should instead wait for a clock tick, not delay a specified amount!
			
			// Heartbeat
			pacifier++;
			if (memoryFull) {
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
