// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2015 Serval Project Inc. and Flinders University, All Rights Reserved.
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
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
/// @file	csma.c
///
/// carrier sense multiple-access code
/// based on the original time division multiplexing code
///

#include <stdarg.h>
#include "radio.h"
#include "csma.h"
#include "timer.h"
#include "packet.h"
#include "golay.h"
#include "crc.h"
#include "flash.h"
#include "flash_layout.h"
#include "at.h"
#include "board.h"
#include "pins_user.h"

#define USE_TICK_YIELD 1

#define MAX_HEADER_LENGTH 9

extern bool last_was_bang;
extern bool tx_buffered_data;

/// a packet buffer for the TDM code
__xdata uint8_t	pbuf[MAX_PACKET_LENGTH];
/// a packet announcement buffer for the TDM code
__xdata uint8_t	hbuf[MAX_HEADER_LENGTH];

/// the maximum data packet size we can fit
__pdata static uint8_t max_data_packet_length;

/// the silence period between transmit windows
/// This is calculated as the number of ticks it would take to transmit
/// two zero length packets
__pdata static uint16_t silence_period;

/// whether we can transmit in the other radios transmit window
/// due to the other radio yielding to us
static __bit bonus_transmit;

/// whether we have yielded our window to the other radio
static __bit transmit_yield;

// activity indication
// when the 16 bit timer2_tick() value wraps we check if we have received a
// packet since the last wrap (ie. every second)
// If we have the green radio LED is held on.
// Otherwise it blinks every 1 seconds. The received_packet flag
// is set for any received packet, whether it contains user data or
// not.
static __bit blink_state;
static __bit received_packet;

/// the latency in 16usec timer2 ticks for sending a zero length packet
__pdata static uint16_t packet_latency;

/// the time in 16usec ticks for sending one byte
__pdata static uint16_t ticks_per_byte;

/// number of 16usec ticks to wait for a preamble to turn into a packet
/// This is set when we get a preamble interrupt, and causes us to delay
/// sending for a maximum packet latency. This is used to make it more likely
/// that two radios that happen to be exactly in sync in their sends
/// will eventually get a packet through and get their transmit windows
/// sorted out
__pdata uint16_t transmit_wait;

/// the long term duty cycle we are aiming for
__pdata uint8_t duty_cycle;

/// the average duty cycle we have been transmitting
__data static float average_duty_cycle;

/// duty cycle offset due to temperature
__pdata uint8_t duty_cycle_offset;

/// set to true if we need to wait for our duty cycle average to drop
static bool duty_cycle_wait;

/// how many ticks we have transmitted for in this TDM round
__pdata static uint16_t transmitted_ticks;

/// the LDB (listen before talk) RSSI threshold
__pdata uint8_t lbt_rssi;

/// test data to display in the main loop. Updated when the tick
/// counter wraps, zeroed when display has happened
__pdata uint8_t test_display;

/// set when we should send a statistics packet on the next round
static __bit send_statistics;

/// set when we should send a MAVLink report pkt
extern bool seen_mavlink;

struct tdm_trailer {
	uint16_t window:13;
	uint16_t command:1;
	uint16_t bonus:1;
	uint16_t resend:1;
};
__pdata struct tdm_trailer trailer;

/// buffer to hold a remote AT command before sending
static bool send_at_command;
static __pdata char remote_at_cmd[AT_CMD_MAXLEN + 1];

/// estimate the flight time for a packet given the payload size
///
/// @param packet_len		payload length in bytes
///
/// @return			flight time in 16usec ticks
static uint16_t flight_time_estimate(__pdata uint8_t packet_len)
{
	return packet_latency + (packet_len * ticks_per_byte);
}

/// called to check temperature
///
static void temperature_update(void)
{
	register int16_t diff;
	if (radio_get_transmit_power() <= 20) {
		duty_cycle_offset = 0;
		return;
	}

	diff = radio_temperature() - MAX_PA_TEMPERATURE;
	if (diff <= 0 && duty_cycle_offset > 0) {
		// under temperature
		duty_cycle_offset -= 1;
	} else if (diff > 10) {
		// getting hot!
		duty_cycle_offset += 10;
	} else if (diff > 5) {
		// well over temperature
		duty_cycle_offset += 5;
	} else if (diff > 0) {
		// slightly over temperature
		duty_cycle_offset += 1;				
	}
	// limit to minimum of 20% duty cycle to ensure link stays up OK
	if ((duty_cycle-duty_cycle_offset) < 20) {
		duty_cycle_offset = duty_cycle - 20;
	}
}


/// blink the radio LED if we have not received any packets
///
static void
link_update(void)
{
	static uint8_t unlock_count, temperature_count;
	if (received_packet) {
		unlock_count = 0;
		received_packet = false;
	} else {
		unlock_count++;
	}
	if (unlock_count < 6) {
		LED_RADIO = LED_ON;
	} else {
		LED_RADIO = blink_state;
		blink_state = !blink_state;
	}
	if (unlock_count > 40) {
		// if we have been unlocked for 20 seconds
		// then start frequency scanning again
		// PGS: Except that with CSMA, we don't scan, but we still reset statistics

		unlock_count = 5;
	}

	if (unlock_count != 0) {
		statistics.average_rssi = (radio_last_rssi() + 3*(uint16_t)statistics.average_rssi)/4;

		// reset statistics when unlocked
		statistics.receive_count = 0;
	}
	if (unlock_count > 5) {
		memset(&remote_statistics, 0, sizeof(remote_statistics));
	}

	send_statistics = 1;

	temperature_count++;
	if (temperature_count == 4) {
		// check every 2 seconds
		temperature_update();
		temperature_count = 0;
	}
}

// dispatch an AT command to the remote system
void
csma_remote_at(void)
{
	memcpy(remote_at_cmd, at_cmd, strlen(at_cmd)+1);
	send_at_command = true;
}

// handle an incoming at command from the remote radio
static void
handle_at_command(__pdata uint8_t len)
{
	if (len < 2 || len > AT_CMD_MAXLEN || 
	    pbuf[0] != (uint8_t)'R' || 
	    pbuf[1] != (uint8_t)'T') {
		// assume its an AT command reply
		register uint8_t i;
		for (i=0; i<len; i++) {
			putchar_r(pbuf[i]);
		}
		return;
	}

	// setup the command in the at_cmd buffer
	memcpy(at_cmd, pbuf, len);
	at_cmd[len] = 0;
	at_cmd[0] = 'A'; // replace 'R'
	at_cmd_len = len;
	at_cmd_ready = true;

	// run the AT command, capturing any output to the packet
	// buffer
	// this reply buffer will be sent at the next opportunity
	printf_start_capture(pbuf, sizeof(pbuf));
	at_command();
	len = printf_end_capture();
	if (len > 0) {
		packet_inject(pbuf, len);
	}
}

// a stack carary to detect a stack overflow
__at(0xFF) uint8_t __idata _canary;

/// main loop for time division multiplexing transparent serial
///
void
csma_serial_loop(void)
{
	__pdata uint16_t last_t = timer2_tick();
	__pdata uint16_t last_link_update = last_t;

	_canary = 42;

	for (;;) {
		__pdata uint8_t	len;
		__pdata uint16_t tnow, tdelta;
		__pdata uint8_t max_xmit;

		if (_canary != 42) {
			panic("stack blown\n");
		}

		if (pdata_canary != 0x41) {
			panic("pdata canary changed\n");
		}

		// give the AT command processor a chance to handle a command
		at_command();

		// set right receive channel
		// PGS: Always the only channel we have for CSMA
		radio_set_channel(0);

		// get the time before we check for a packet coming in
		tnow = timer2_tick();

		// see if we have received a packet
		if (radio_receive_packet(&len, pbuf)) {

			// update the activity indication
			received_packet = true;
			
			// update filtered RSSI value and packet stats
			statistics.average_rssi = (radio_last_rssi() + 7*(uint16_t)statistics.average_rssi)/8;
			statistics.receive_count++;
			
			// we're not waiting for a preamble
			// any more
			transmit_wait = 0;

			if (len < 2) {
				// not a valid packet. We always send
				// two control bytes at the end of every packet
				continue;
			}

			// extract control bytes from end of packet
			memcpy(&trailer, &pbuf[len-sizeof(trailer)], sizeof(trailer));
			len -= sizeof(trailer);

			if (trailer.window == 0 && len != 0) {
				// its a control packet
				if (len == sizeof(struct statistics)) {
					memcpy(&remote_statistics, pbuf, len);
				}

				// Count stats packets separately
				statistics.receive_count--;
				statistics.receive_count_statspkt++;
			} else if (trailer.window != 0) {
				last_t = tnow;
				
				if (trailer.command == 1) {
					handle_at_command(len);
				} else {		
					if (!at_mode_active) {
						// PGS: The following framing allows servald and friends to accurately identify the
						// framing of a packet
						// Indicate radio frame here, even if frame is empty.

						// Write packet body out first, so that its location can be deduced as soon as the
						// frame has been received, without needing to read in the packet body.
						if (len != 0) {
						// its user data - send it out
						// the serial port
						//printf("rcv(%d,[", len);
						LED_ACTIVITY = LED_ON;
						
						serial_write_buf(pbuf, len);
						LED_ACTIVITY = LED_OFF;
						//printf("]\n");
						}
						
						hbuf[0]=0xaa;
						hbuf[1]=0x55;
						// RSSI of THIS frame
						hbuf[2]=radio_last_rssi();
						// Remote average RSSI
						hbuf[3]=remote_statistics.average_rssi;
						// Radio temperature		
						hbuf[4]=radio_temperature();
						// length of frame
						hbuf[5]=len;						
						// RX buffer space
						hbuf[6]=serial_read_space_bytes()&0xff;
						hbuf[7]=serial_read_space_bytes()>>8;
						hbuf[8]=0x55;
						serial_write_buf(hbuf, 8+1);
					}
				}
			}
			continue;
		}

		// see how many 16usec ticks have passed and update
		// the tdm state machine. We re-fetch tnow as a bad
		// packet could have cost us a lot of time.
		tnow = timer2_tick();
		tdelta = tnow - last_t;
		last_t = tnow;

		// update link status every 0.5s
		if (tnow - last_link_update > 32768) {
			link_update();
			last_link_update = tnow;

			if (!at_mode_active) {
				uint8_t i;
				// Also report on the status of the GPIOs
				// Wrap in UTF-8 2-byte codes for { and }
				// for ease of human readability, while remaining
				// unambigious for machine decoding
				putchar_r(0xc1);
				putchar_r(0xbb);				
				for(i=0;i<6;i++) {
#if PIN_MAX > 0
					if(pins_user_get_io(i) == PIN_INPUT)
						putchar_r(0x30+pins_user_get_adc(i));
					else
						if (pins_user_get_value(i))
							putchar_r('X');
						else
							putchar_r('x');
#else
					putchar_r(0xbd);
#endif
				}
				putchar_r(0xc1);
				putchar_r(0xbd);				

				// XXX - We need to check GPIOs for regulatory domain selection
				// Indicate board frequency
				putchar_r('0'+(g_board_frequency>>4));
				putchar_r('0'+(g_board_frequency&0xf));
				// CR LF for convenience
				putchar_r(0x0d);
				putchar_r(0x0a);
			}
		}

		if (!received_packet &&
		    radio_preamble_detected() ||
		    radio_receive_in_progress()) {
			// a preamble has been detected. Don't
			// transmit for a while
			transmit_wait = packet_latency;
			continue;
		}

		// sample the background noise when it is out turn to
		// transmit, but we are not transmitting,
		// averaged over around 4 samples
		statistics.average_noise = (radio_current_rssi() + 3*(uint16_t)statistics.average_noise)/4;

		if (duty_cycle_wait) {
			// we're waiting for our duty cycle to drop
			continue;
		}

		// PGS: Now that we are basically using a CSMA protocol, we just always allow sending the
		// maximum number of bytes.
		max_xmit = max_data_packet_length;

		// ask the packet system for the next packet to send
		if (send_at_command && 
		    max_xmit >= strlen(remote_at_cmd)) {
			// send a remote AT command
			len = strlen(remote_at_cmd);
			memcpy(pbuf, remote_at_cmd, len);
			trailer.command = 1;
			send_at_command = false;
		} else {
			// Only send packets if we have bytes pending, and the noise level is low enough.
			// Ideally we should use a dynamic LBT regieme instead of a fixed point like this,
			// but it will probably be okay.
			if (statistics.average_noise < 70)
				if (tx_buffered_data&&((radio_current_rssi() < lbt_rssi))) {
					// get a packet from the serial port
					len = packet_get_next(max_xmit, pbuf);
					trailer.command = packet_is_injected();
					
					if (len > max_data_packet_length) {
						panic("oversized tdm packet");
					}
					
					// Clear TX gate
					tx_buffered_data = 0;
					
					trailer.resend = packet_is_resend();
					
					// PGS: Trailer window is meaningless in CSMA mode,
					// except to indicate that the packet is data, not
					// statistics.
					trailer.window = 1;
					
					// set right transmit channel
					// PGS: Always channel 0 for CSMA
					radio_set_channel(0);
					
					memcpy(&pbuf[len], &trailer, sizeof(trailer));
					
					// show the user that we're sending real data
					LED_ACTIVITY = LED_ON;
					
					// after sending a packet leave a bit of time before
					// sending the next one. The receivers don't cope well
					// with back to back packets
					transmit_wait = packet_latency;
					
					// if we're implementing a duty cycle, add the
					// transmit time to the number of ticks we've been transmitting
					if ((duty_cycle - duty_cycle_offset) != 100) {
						transmitted_ticks += flight_time_estimate(len+sizeof(trailer));
					}

					// start transmitting the packet
					radio_transmit(len + sizeof(trailer), pbuf, 10000 );
					
					// PGS DEBUG: Display a character each time we send a packet.
					hbuf[0]='.';
					serial_write_buf(hbuf, 1);
					
					// set right receive channel
					// PGS: Always channel 0 for CSMA
					radio_set_channel(0);
					
					// re-enable the receiver
					radio_receiver_on();
					
					LED_ACTIVITY = LED_OFF;
				}
		}
	}
}

// initialise the CSMA subsystem
void
csma_init(void)
{
	__pdata uint8_t air_rate = radio_air_rate();

#define REGULATORY_MAX_WINDOW (((1000000UL/16)*4)/10)
#define LBT_MIN_TIME_USEC 5000

	// calculate how many 16usec ticks it takes to send each byte
	ticks_per_byte = (8+(8000000UL/(air_rate*1000UL)))/16;

	// calculate the minimum packet latency in 16 usec units
	// we initially assume a preamble length of 40 bits, then
	// adjust later based on actual preamble length. This is done
	// so that if one radio has antenna diversity and the other
	// doesn't, then they will both using the same TDM round timings
	packet_latency = (8+(10/2)) * ticks_per_byte + 13;

	if (feature_golay) {
		max_data_packet_length = (MAX_PACKET_LENGTH/2) - (6+sizeof(trailer));

		// golay encoding doubles the cost per byte
		ticks_per_byte *= 2;

		// and adds 4 bytes
		packet_latency += 4*ticks_per_byte;
	} else {
		max_data_packet_length = MAX_PACKET_LENGTH - sizeof(trailer);
	}

	// set the silence period to two times the packet latency
        silence_period = 2*packet_latency;

	// now adjust the packet_latency for the actual preamble
	// length, so we get the right flight time estimates, while
	// not changing the round timings
	packet_latency += ((settings.preamble_length-10)/2) * ticks_per_byte;

	// tell the packet subsystem our max packet size, which it
	// needs to know for MAVLink packet boundary detection
	// PGS: With CSMA, this is always the maximum
	packet_set_max_xmit(max_data_packet_length);

}


/// report tdm timings
///
void 
csma_report_timing(void)
{
	printf("silence_period: %u\n", (unsigned)silence_period); delay_msec(1);
	printf("max_data_packet_length: %u\n", (unsigned)max_data_packet_length); delay_msec(1);
}

