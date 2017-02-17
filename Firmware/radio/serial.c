// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2011 Michael Smith, All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  o Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  o Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.
//

///
/// @file	serial.c
///
/// MCS51 Serial port driver with flow control and AT command
/// parser integration.
///

#include "flash_layout.h"
#include "serial.h"
#include "packet.h"
#include "i2c.h"

// Serial rx/tx buffers.
//
// Note that the rx buffer is much larger than you might expect
// as we need the receive buffer to be many times larger than the
// largest possible air packet size for efficient TDM. Ideally it
// would be about 16x larger than the largest air packet if we have
// 8 TDM time slots
//

#define RX_BUFF_MAX 512
#define TX_BUFF_MAX 512

__xdata uint8_t rx_buf[RX_BUFF_MAX] = {0};
__xdata uint8_t tx_buf[TX_BUFF_MAX] = {0};

// TX gate / ! escape state
extern bool last_was_bang=0;
extern bool tx_buffered_data=0;

// FIFO insert/remove pointers
static volatile __pdata uint16_t				rx_insert, rx_remove;
static volatile __pdata uint16_t				tx_insert, tx_remove;

// count of number of bytes we are allowed to send due to a RTS low reading
static uint8_t rts_count;

// flag indicating the transmitter is idle
static volatile bool			tx_idle;

// FIFO status
#define BUF_NEXT_INSERT(_b)	((_b##_insert + 1) == sizeof(_b##_buf)?0:(_b##_insert + 1))
#define BUF_NEXT_REMOVE(_b)	((_b##_remove + 1) == sizeof(_b##_buf)?0:(_b##_remove + 1))
#define BUF_FULL(_b)	(BUF_NEXT_INSERT(_b) == (_b##_remove))
#define BUF_NOT_FULL(_b)	(BUF_NEXT_INSERT(_b) != (_b##_remove))
#define BUF_EMPTY(_b)	(_b##_insert == _b##_remove)
#define BUF_NOT_EMPTY(_b)	(_b##_insert != _b##_remove)
#define BUF_USED(_b)	((_b##_insert >= _b##_remove)?(_b##_insert - _b##_remove):(sizeof(_b##_buf) - _b##_remove) + _b##_insert)
#define BUF_FREE(_b)	((_b##_insert >= _b##_remove)?(sizeof(_b##_buf) + _b##_remove - _b##_insert):_b##_remove - _b##_insert)

// FIFO insert/remove operations
//
// Note that these are nominally interrupt-safe as only one of each
// buffer's end pointer is adjusted by either of interrupt or regular
// mode code.  This is violated if printing from interrupt context,
// which should generally be avoided when possible.
//
#define BUF_INSERT(_b, _c)	do { _b##_buf[_b##_insert] = (_c); \
		_b##_insert = BUF_NEXT_INSERT(_b); } while(0)
#define BUF_REMOVE(_b, _c)	do { (_c) = _b##_buf[_b##_remove]; \
		_b##_remove = BUF_NEXT_REMOVE(_b); } while(0)
#define BUF_PEEK(_b)	_b##_buf[_b##_remove]
#define BUF_PEEK2(_b)	_b##_buf[BUF_NEXT_REMOVE(_b)]
#define BUF_PEEKX(_b, offset)	_b##_buf[(_b##_remove+offset) % sizeof(_b##_buf)]

static void			_serial_write(register uint8_t c);
static void			serial_restart(void);
static void serial_device_set_speed(register uint8_t speed);

// save and restore serial interrupt. We use this rather than
// __critical to ensure we don't disturb the timer interrupt at all.
// minimal tick drift is critical for TDM
#define ES0_SAVE_DISABLE __bit ES_saved = ES0; ES0 = 0
#define ES0_RESTORE ES0 = ES_saved

// threshold for considering the rx buffer full
#define SERIAL_CTS_THRESHOLD_LOW  17
#define SERIAL_CTS_THRESHOLD_HIGH 34

void
serial_interrupt(void) __interrupt(INTERRUPT_UART0)
{
	register uint8_t	c;
	static __xdata short eeprom_address;
						

	// check for received byte first
	if (RI0) {
		// acknowledge interrupt and fetch the byte immediately
		RI0 = 0;
		c = SBUF0;

		// if AT mode is active, the AT processor owns the byte
		if (at_mode_active) {
			// If an AT command is ready/being processed, we would ignore this byte
			if (!at_cmd_ready) {
				at_input(c);
			}
		} else {
			// run the byte past the +++ detector
			at_plus_detector(c);

			// PGS: To enforce packet boundaries where we want them, we escape
			// '!'.  '!!' means send buffered serial data.  '!' followed by '.'
			// inserts a '!' into the serial buffer.
			if (c=='!') {
				if (last_was_bang) {
					tx_buffered_data=1;
					last_was_bang=0;
				} else {
					last_was_bang=1;
				}
			} else if ((c=='C') && last_was_bang ) {
				// clear TX buffer
				last_was_bang=0;
				while (BUF_NOT_EMPTY(rx)) {
					BUF_REMOVE(rx,c);
				}
			} else if ((c>='a') &&(c<='z') && last_was_bang ) {
				last_was_bang=0;
#if PIN_MAX > 0
				// I2C debug functions
		        switch (c) {
			case 'p': eeprom_poweron(); break;
			case 'o': eeprom_poweroff(); break;
			case 'a': i2c_start(); break;
			case 'z': i2c_stop(); break; 
			case 's': i2c_clock_high(); break;
			case 'x': i2c_clock_low(); break;
			case 'd': i2c_data_high(); break;
			case 'c': i2c_data_low(); break;
			case 'y':
				// Disable write-protect temporarily
				// (writing to EEPROM reasserts it automatically)
				pins_user_set_io(5,PIN_OUTPUT);
				pins_user_set_value(5,0);
				break;
			case 'f': i2c_clock_low(); i2c_delay();
				  i2c_data_high(); i2c_delay();
				  i2c_clock_high(); i2c_delay();
				  i2c_clock_low(); i2c_delay();
				  break;
			case 'v': i2c_clock_low(); i2c_delay();
				  i2c_data_low(); i2c_delay();
				  i2c_clock_high(); i2c_delay();
				  i2c_clock_low(); i2c_delay();
				  break;
			case 'b': case 'm': case 'n':
				// Adjust where to read or write data in EEPROM
				if (c=='b') eeprom_address-=0x100;
				if (c=='m') eeprom_address+=0x100;
				if (c=='n') eeprom_address+=0x10;
				if (eeprom_address<0) eeprom_address+=0x800;
				if (eeprom_address>=0x800) eeprom_address-=0x800;
				printfl("EPRADDR=$%x\r\n",eeprom_address);
				break;
			case 'w':
				// Write a page of data to EEPROM.
				// We copy the first 16 bytes from the TX buffer
				// to write.
				eeprom_poweron();
				printfl("\r\n");
				{
					// Copy bytes from TX buffer
					char i;
					for(i=0;i<16;i++)
						eeprom_data[i]=tx_buf[i];
					if (eeprom_write_page(eeprom_address))
						printfl("WRITE ERROR\r\n");
					else
						printf("EEPROM WRITTEN\r\n");
					
				}
				eeprom_poweroff();
				// Re-enable write-protect
				pins_user_set_io(5,PIN_INPUT);
				pins_user_set_value(5,1);
				break;
			}
#endif				
			} else if ((c=='E') && last_was_bang ) {
				// Dump EEPROM contents
				{
					unsigned char count=0;
					eeprom_poweron();
					printfl("\r\n");
					while(1)
						{
							char i;
							printfl("EPR:%x : ",eeprom_address);
							i=eeprom_read_page(eeprom_address);
							if (i) printfl("READ ERROR #%d",i);
							else {
								for(i=0;i<16;i++)
									printfl(" %x",eeprom_data[i]);
							}
							printfl("\r\n");
							eeprom_address+=16;
							if (eeprom_address>=0x800) eeprom_address=0;
							
							count+=16;
							if (count==0x80) break;
						}

					eeprom_poweroff();
					
					last_was_bang=0;
				}
			} else if ((c=='F') && last_was_bang ) {
				// Identify radio firmware by series of checksums of flash
				last_was_bang=0;
				flash_report_summary();
			} else if ((c=='H') && last_was_bang ) {
				last_was_bang=0;
				param_set(PARAM_TXPOWER,25);
			} else if ((c=='Z') && last_was_bang ) {
				// Trigger a reset of radio by software (like ATZ)
				RSTSRC |= (1 << 4);
	                        for (;;)
       	                         ;
			} else if ((c=='L') && last_was_bang ) {
				last_was_bang=0;
				param_set(PARAM_TXPOWER,1);
			} else if ((c=='R') && last_was_bang ) {
				// Reset radio to default settings (like AT&F)
				param_default();
			} else if ((c=='V') && last_was_bang ) {
				// Provide version info, to allow quick detection of CSMA
				// firmware
				putchar_r('1');
			} else if ((c=='.') && last_was_bang ) {
				last_was_bang=0;
				// Insert escaped ! into serial RX buffer
				if (BUF_NOT_FULL(rx)) {
					BUF_INSERT(rx, '!');
				} else {
					if (errors.serial_rx_overflow != 0xFFFF) {
						errors.serial_rx_overflow++;
					}
				}
			} else if (last_was_bang) {
				// Unknown ! command
				last_was_bang=0;
				putchar_r('E');
			} else {
				// Character to put in TX buffer
				if (BUF_NOT_FULL(rx)) {
					BUF_INSERT(rx, c);
				} else {
					if (errors.serial_rx_overflow != 0xFFFF) {
						errors.serial_rx_overflow++;
					}
				}				
			}
#ifdef SERIAL_CTS
			if (BUF_FREE(rx) < SERIAL_CTS_THRESHOLD_LOW) {
				SERIAL_CTS = true;
			}
#endif
		}
	}

	// check for anything to transmit
	if (TI0) {
		// acknowledge the interrupt
		TI0 = 0;

		// look for another byte we can send
		if (BUF_NOT_EMPTY(tx)) {
#ifdef SERIAL_RTS
		if (feature_rtscts) {
				if (SERIAL_RTS && !at_mode_active) {
						if (rts_count == 0) {
								// the other end doesn't have room in
								// its serial buffer
								tx_idle = true;
								return;
						}
						rts_count--;
				} else {
								rts_count = 8;
				}
		}
#endif
			// fetch and send a byte
			BUF_REMOVE(tx, c);
			SBUF0 = c;
		} else {
			// note that the transmitter requires a kick to restart it
			tx_idle = true;
		}
	}
}


/// check if RTS allows us to send more data
///
void
serial_check_rts(void)
{
	if (BUF_NOT_EMPTY(tx) && tx_idle) {
		serial_restart();
	}
}

void
serial_init(register uint8_t speed)
{
	// disable UART interrupts
	ES0 = 0;

	// reset buffer state, discard all data
	rx_insert = 0;
	rx_remove = 0;
	tx_insert = 0;
	tx_remove = 0;
	tx_idle = true;

	// configure timer 1 for bit clock generation
	TR1 	= 0;				// timer off
	TMOD	= (TMOD & ~0xf0) | 0x20;	// 8-bit free-running auto-reload mode
	serial_device_set_speed(speed);		// device-specific clocking setup
	TR1	= 1;				// timer on

	// configure the serial port
	SCON0	= 0x10;				// enable receiver, clear interrupts

#ifdef SERIAL_CTS
	// setting SERIAL_CTS low tells the other end that we have
	// buffer space available
	SERIAL_CTS = false;
#endif

	// re-enable UART interrupts
	ES0 = 1;
}

bool
serial_write(register uint8_t c)
{
	if (serial_write_space() < 1)
		return false;

	_serial_write(c);
	return true;
}

static void
_serial_write(register uint8_t c) __reentrant
{
	ES0_SAVE_DISABLE;

	// if we have space to store the character
	if (BUF_NOT_FULL(tx)) {

		// queue the character
		BUF_INSERT(tx, c);

		// if the transmitter is idle, restart it
		if (tx_idle)
			serial_restart();
	} else if (errors.serial_tx_overflow != 0xFFFF) {
		errors.serial_tx_overflow++;
	}

	ES0_RESTORE;
}

// write as many bytes as will fit into the serial transmit buffer
// if encryption turned on, decrypt the packet.
void
serial_write_buf(__xdata uint8_t * buf, __pdata uint8_t count)
{
	__pdata uint16_t space;
	__pdata uint8_t n1;

	if (count == 0) {
		return;
	}
  
	// discard any bytes that don't fit. We can't afford to
	// wait for the buffer to drain as we could miss a frequency
	// hopping transition
	space = serial_write_space();	
	if (count > space) {
		count = space;
		if (errors.serial_tx_overflow != 0xFFFF) {
			errors.serial_tx_overflow++;
		}
	}

	// write to the end of the ring buffer
	n1 = count;
	if (n1 > sizeof(tx_buf) - tx_insert) {
		n1 = sizeof(tx_buf) - tx_insert;
	}
	memcpy(&tx_buf[tx_insert], buf, n1);
	buf += n1;
	count -= n1;
	__critical {
		tx_insert += n1;
		if (tx_insert >= sizeof(tx_buf)) {
			tx_insert -= sizeof(tx_buf);
		}
	}

	// add any leftover bytes to the start of the ring buffer
	if (count != 0) {
		memcpy(&tx_buf[0], buf, count);
		__critical {
			tx_insert = count;
		}		
	}
	__critical {
		if (tx_idle) {
			serial_restart();
		}
	}
}

uint16_t
serial_write_space(void)
{
	register uint16_t ret;
	ES0_SAVE_DISABLE;
	ret = BUF_FREE(tx);
	ES0_RESTORE;
	return ret;
}

static void
serial_restart(void)
{
#ifdef SERIAL_RTS
	if (feature_rtscts && SERIAL_RTS && !at_mode_active) {
		// the line is blocked by hardware flow control
		return;
	}
#endif
	// generate a transmit-done interrupt to force the handler to send another byte
	tx_idle = false;
	TI0 = 1;
}

uint8_t
serial_read(void)
{
	register uint8_t	c;

	ES0_SAVE_DISABLE;

	if (BUF_NOT_EMPTY(rx)) {
		BUF_REMOVE(rx, c);
	} else {
		c = '\0';
	}

#ifdef SERIAL_CTS
	if (BUF_FREE(rx) > SERIAL_CTS_THRESHOLD_HIGH) {
		SERIAL_CTS = false;
	}
#endif

	ES0_RESTORE;

	return c;
}

uint8_t
serial_peek(void)
{
	register uint8_t c;

	ES0_SAVE_DISABLE;
	c = BUF_PEEK(rx);
	ES0_RESTORE;

	return c;
}

uint8_t
serial_peek2(void)
{
	register uint8_t c;

	ES0_SAVE_DISABLE;
	c = BUF_PEEK2(rx);
	ES0_RESTORE;

	return c;
}

uint8_t
serial_peekx(uint16_t offset)
{
	register uint8_t c;

	ES0_SAVE_DISABLE;
	c = BUF_PEEKX(rx, offset);
	ES0_RESTORE;

	return c;
}

// read count bytes from the serial buffer. This implementation
// tries to be as efficient as possible, while disabling interrupts
// for as short a time as possible
bool
serial_read_buf(__xdata uint8_t * buf, __pdata uint8_t count)
{
	__pdata uint16_t n1;
	// the caller should have already checked this, 
	// but lets be sure
	if (count > serial_read_available()) {
		return false;
	}
	// see how much we can copy from the tail of the buffer
	n1 = count;
	if (n1 > sizeof(rx_buf) - rx_remove) {
		n1 = sizeof(rx_buf) - rx_remove;
	}
	memcpy(buf, &rx_buf[rx_remove], n1);
	count -= n1;
	buf += n1;
	// update the remove marker with interrupts disabled
	__critical {
		rx_remove += n1;
		if (rx_remove >= sizeof(rx_buf)) {
			rx_remove -= sizeof(rx_buf);
		}
	}
	// any more bytes to do?
	if (count > 0) {
		memcpy(buf, &rx_buf[0], count);
		__critical {
			rx_remove = count;
		}		
	}

#ifdef SERIAL_CTS
	__critical {
		if (BUF_FREE(rx) > SERIAL_CTS_THRESHOLD_HIGH) {
			SERIAL_CTS = false;
		}
	}
#endif
	return true;
}

uint16_t
serial_read_available(void)
{
	register uint16_t ret;
	ES0_SAVE_DISABLE;
	ret = BUF_USED(rx);
	ES0_RESTORE;
	return ret;
}

// return available space in rx buffer as a percentage
uint8_t
serial_read_space(void)
{
	uint16_t space = sizeof(rx_buf) - serial_read_available();
	space = (100 * (space/8)) / (sizeof(rx_buf)/8);
	return space;
}

uint16_t
serial_read_space_bytes(void)
{
        return sizeof(rx_buf) - serial_read_available();
}


void
putchar_r(char c) __reentrant
{
	if (c == '\n')
		_serial_write('\r');
	_serial_write(c);
}

void puts_r(char *s)
{
	while(s++) putchar_r(*s);
	putchar_r('\n');
}


///
/// Table of supported serial speed settings.
/// the table is looked up based on the 'one byte'
/// serial rate scheme that APM uses. If an unsupported
/// rate is chosen then 57600 is used
///
static const __code struct {
	uint8_t rate;
	uint8_t th1;
	uint8_t ckcon;
} serial_rates[] = {
	{1,   0x2C, 0x02}, // 1200
	{2,   0x96, 0x02}, // 2400
	{4,   0x2C, 0x00}, // 4800
	{9,   0x96, 0x00}, // 9600
	{19,  0x60, 0x01}, // 19200
	{38,  0xb0, 0x01}, // 38400
	{57,  0x2b, 0x08}, // 57600 - default
	{115, 0x96, 0x08}, // 115200
	{230, 0xcb, 0x08}, // 230400
};

//
// check if a serial speed is valid
//
bool 
serial_device_valid_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			return true;
		}
	}
	return false;
}

static 
void serial_device_set_speed(register uint8_t speed)
{
	uint8_t i;
	uint8_t num_rates = ARRAY_LENGTH(serial_rates);

	for (i = 0; i < num_rates; i++) {
		if (speed == serial_rates[i].rate) {
			break;
		}
	}
	if (i == num_rates) {
		i = 6; // 57600 default
	}

	// set the rates in the UART
	TH1 = serial_rates[i].th1;
	CKCON = (CKCON & ~0x0b) | serial_rates[i].ckcon;

	// tell the packet layer how fast the serial link is. This is
	// needed for packet framing timeouts
	packet_set_serial_speed(speed*125UL);	
}

