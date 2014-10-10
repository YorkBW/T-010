/*
 * T_010.c
 *
 * Created: 9/6/2014 4:20:24 PM
 *  Author: Brent
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include "usbdrv.h"
#include "i2cmaster.h"

#define F_CPU 12000000L
#include <util/delay.h>


// General configuration
#define USB_CONNECTED_PORT_IN	PINB
#define USB_CONNECTED_PIN_IN	PINB2
#define USB_CONNECTED			(USB_CONNECTED_PORT_IN & _BV(USB_CONNECTED_PIN_IN))

// times in milliseconds
#define DATA_TICK            50
#define HEARTBEAT_TICK     2000
#define HEARTBEAT_ON_TIME   100


// I2C device definitions
//const uint8_t INA219_DEVICE       = 0x40 << 1;
const uint8_t MPL115A2_DEVICE        = 0xC0;
const uint8_t MPL115A2_READPRESMSB   = 0x00;
const uint8_t MPL115A2_READPRESLSB   = 0x01;
const uint8_t MPL115A2_READTEMPMSB   = 0x02;
const uint8_t MPL115A2_READTEMPLSB   = 0x03;
const uint8_t MPL115A2_READCOEFFIRST = 0x04;
const uint8_t MPL115A2_READCOEFLAST  = 0x0B;
const uint8_t MPL115A2_STARTCONV     = 0x12;



// SPI device definitions
const int ALTCOR = 192;	// sealevel altitude correction (in mb * 10)

// USB interface definitions
#define USB_STATUS    0
#define USB_DATA_DUMP 1
#define USB_PRESSURE  2


// Global variables
#define DATA_BUF_SZ 16
static uchar dataBuf[DATA_BUF_SZ];

uchar myUSICR;

#define DEV_GOOD_MPL115A2		0
//#define DEV_GOOD_SCP1000	1
uchar devicesGood;

int16_t a0, b1, b2, c12;
int16_t lastPres;


// TEMP: Forward function declarations
void startMPL115A2Read(void);
int16_t readMPL115A2Pressure(void);


// -------------------------
//  USB interface functions
// -------------------------

USB_PUBLIC uchar usbFunctionSetup(uchar data[8])
// Responds to USB messages
{
	//int temp;
	//uint16_t utemp;
	
	usbRequest_t *rq = (void *)data; // cast data to correct type
	
	switch(rq->bRequest) { // custom command is in the bRequest field

		case USB_STATUS:
			usbMsgPtr = (usbMsgPtr_t)(&devicesGood);
			return 1;

		case USB_DATA_DUMP: // send data to PC
			usbMsgPtr = (usbMsgPtr_t)dataBuf;
			return DATA_BUF_SZ;

		case USB_PRESSURE:
			startMPL115A2Read();
			_delay_ms(3);
			lastPres = readMPL115A2Pressure();
			usbMsgPtr = (usbMsgPtr_t)(&lastPres);
			return sizeof(lastPres);

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

// ----------------------
//  I2C device functions
// ----------------------

uint8_t initMPL115A2(void)
// Initialize the MPL115A2 pressure sensor, return 0 if bad comms
{
	uchar coef;
	
// Init comms
	if (i2c_start(MPL115A2_DEVICE | I2C_WRITE) != 0) return 0;
	
// Read coefficients
	i2c_write(MPL115A2_READCOEFFIRST);
	i2c_rep_start(MPL115A2_DEVICE | I2C_READ);
	for (coef=MPL115A2_READCOEFFIRST; coef<=MPL115A2_READCOEFLAST; coef++) {
		dataBuf[coef - MPL115A2_READCOEFFIRST] = i2c_readAck();
	}
	
	i2c_stop();
	
	a0  = (int16_t)((dataBuf[0] << 8) | dataBuf[1]);	// 3 fractional bits
	b1  = (int16_t)((dataBuf[2] << 8) | dataBuf[3]);	// 13 fractional bits
	b2  = (int16_t)((dataBuf[4] << 8) | dataBuf[5]);	// 14 fractional bits
	c12 = (int16_t)((dataBuf[6] << 8) | dataBuf[7]);    // 9 dec pt zero pad

	return 1;	// device good
}


void startMPL115A2Read(void)
// Start ADC conversion on MPL115A2 (needs up to 3ms to complete)
{
	i2c_start_wait(MPL115A2_DEVICE | I2C_WRITE);
	i2c_write(MPL115A2_STARTCONV);
	i2c_write(0);
	i2c_stop();
}


int16_t readMPL115A2Pressure(void)
// Read data from MPL115A2 and return pressure in kPa (with 4-bit fractional part)
{
	uint16_t Padc, Tadc;
	int32_t c12x2, a1, a1x1, y1, a2x2, PComp;
	
	i2c_start_wait(MPL115A2_DEVICE | I2C_WRITE);
	i2c_write(0);
	i2c_rep_start(MPL115A2_DEVICE | I2C_READ);
	Padc = (i2c_readAck() << 2) + (i2c_readAck() >> 6);
	Tadc = (i2c_readAck() << 2) + (i2c_readNak() >> 6);
	i2c_stop();

	c12x2 = (((int32_t)c12) * Tadc) >> 11;	// c12x2 = c12 * Tadc
	a1    = (int32_t)b1 + c12x2;			// a1    = b1  + c12x2
	a1x1  = a1 * Padc;						// a1x1  = a1  * Padc
	y1    = (((int32_t)a0) << 10) + a1x1;	// y1    = a0  + a1x1
	a2x2  = (((int32_t)b2) * Tadc) >> 1;	// a2x2  = b2  * Tadc
	PComp = (y1 + a2x2) >> 9;				// PComp = y1  + a2x2

	return (int16_t)((((int32_t)PComp) * 1041) >> 14) + 800;
}



// ----------------------
//  SPI device functions
// ----------------------
#if 0
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

void SCPWriteReg(uint8_t reg, uint8_t val)
// Write a register for SCP1000
{
	reg <<= 2;
	reg |= 2;
	PORTB &= !_BV(PORTB4);			// set chip select (low is set)
	spiByte(reg);	spiByte(val);	PORTB |= _BV(PORTB4);			// clear chip select (high is clear)
}
uint8_t SCPReadReg8(uint8_t reg)// Read an 8-bit register for SCP1000
{	uint8_t retVal;		reg <<= 2;
	reg &= 0xfc;
	PORTB &= !_BV(PORTB4);			// set chip select (low is set)
	spiByte(reg);	retVal = spiByte(0);	PORTB |= _BV(PORTB4);			// clear chip select (high is clear)
	
	return retVal;
}uint16_t SCPReadReg16(uint8_t reg)// Read a 16-bit register for SCP1000
{	uint16_t retVal;		reg <<= 2;
	reg &= 0xfc;
	PORTB &= !_BV(PORTB4);			// set chip select (low is set)
	spiByte(reg);	retVal = (spiByte(0) << 8) | spiByte(0);	PORTB |= _BV(PORTB4);			// clear chip select (high is clear)
	
	return retVal;
}
uint8_t initSCP1000(void)
// Initialize the SCP1000 device, return 0 if bad comms
{
	SCPWriteReg(3, 0x0a);	// select high-precision mode//	SCPReadReg8(0x1f);		// dummy read of pressure register to clear DRDY//	SCPReadReg16(0x20);
	return 1;	// device good
}

uint16_t readSCPPressure(void)
// Read pressure from SCP1000 in 10s of mbar
{
	/*uint8_t status;*/
	uint8_t presH;
	uint16_t pres;
	
    /*status =*/ SCPReadReg8(0x07);						// read status register    presH = SCPReadReg8(0x1f);                  // read pressure register (high bits)    presH &= 0x07;                              // only lower 3 bits are significant	pres = presH;	pres <<= 13;    pres |= SCPReadReg16(0x20) >> 3;                // read lower bits of pressure (in mb * 10)	pres /= 5;    pres += ALTCOR;		return pres;}

int readSCPTemperature(void)
// Read temperature from SCP1000 in 1/100 deg F
{
	/*uint8_t status;*/
	uint16_t rawtemp;
	int TempC, TempF;
	
	/*status =*/ SCPReadReg8(0x07);						// read status register	rawtemp = SCPReadReg16(0x21);            // read temperature register		TempC = rawtemp * 10 / 2;                // compute temperature in °C x 100	TempF = TempC * 9 / 5 + 3200;		return TempF;}
#endif

// --------------
//  Main program
// --------------

int main(void)
{
	uchar i;
	uint16_t pacifier = 0;
	uchar usbConnected, usbConnectedLast = 0;
	uint16_t PACIFIER_OFF_TGT = HEARTBEAT_TICK / DATA_TICK;
	uint16_t PACIFIER_ON_TGT = PACIFIER_OFF_TGT - (HEARTBEAT_ON_TIME / DATA_TICK);

// Enable 1 sec watchdog timer
	wdt_enable(WDTO_1S);

// Set up pins
								// PA0: Crystal
								// PA1: Crystal
								// PA2: !RESET
								// PB0: spare
								// PB1: spare
	DDRB &= ~_BV(DDB2);			// PB2: USB connection present flag
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
								// PD4: unused
								// PD5: unused
	DDRD |= _BV(DDD6);			// PD6: Status LED (output)

// Initialize software I2C
	i2c_init();

// Enable SPI mode for USI (hardware SPI)
	// Configure USI to 3-wire master mode with overflow interrupt
	myUSICR = /*_BV(USIOIE) |*/ _BV(USIWM0) | _BV(USICS1) | /*_BV(USICS0) |*/ _BV(USICLK);
	USICR = myUSICR;
	
// Initialize all bus devices
	devicesGood = 0;
	devicesGood |= initMPL115A2() << DEV_GOOD_MPL115A2;
	//devicesGood |= initSCP1000() << DEV_GOOD_SCP1000;

// Initialize USB
	// CHECK whether this is working as expected for start-up when not connected to USB
	usbInit();
	usbDeviceDisconnect(); // enforce re-enumeration
	for (i=0; i<250; i++) { // wait 500 ms
		wdt_reset(); // keep the watchdog happy
		_delay_ms(2);
	}
	usbDeviceConnect();
	sei(); // Enable interrupts after re-enumeration

// Main loop
	while (1) {
		// Reset watchdog
		wdt_reset();
		
		// If connected to USB, poll for messages (and keep status light on solid)
		usbConnected = USB_CONNECTED;
		if (usbConnected) {
			usbPoll();
			PORTD |= _BV(PORTD6); // light status LED
			//if (++pacifier > 60000) PIND |= _BV(PIND6);
		}
		
		// If NOT connected to USB, collect data (and flash status light as heartbeat)
		else {
			if (usbConnectedLast) PORTD &= ~_BV(PORTD6); // turn status LED off if just removed
			_delay_ms(50);  // should instead wait for a clock tick, not delay a specified amount!
			
			// Heartbeat
			if (++pacifier > PACIFIER_ON_TGT) {
				if (pacifier > PACIFIER_OFF_TGT) {
					PORTD &= ~_BV(PORTD6); // turn status LED off
					pacifier = 0;
				}
				else PORTD |= _BV(PORTD6); // turn status LED on
			}
		}
		
		// Read I2C device (voltage and current)
		
		// Read SPI device (temperature and pressure)

		usbConnectedLast = usbConnected;
	}
	
	return 0;  // never reached
}
