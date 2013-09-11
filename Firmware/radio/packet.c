// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
// Copyright (c) 2013 Paul Gardner-Stephen, All Rights Reserved
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
/// @file	packet.c
///
/// packet handling code
///

#include <stdarg.h>
#include "radio.h"
#include "packet.h"
#include "timer.h"

static __bit last_sent_is_resend;
static __bit last_recv_is_resend;
static __bit force_resend;

static __xdata uint8_t last_received[MAX_PACKET_LENGTH];
static __xdata uint8_t last_sent[MAX_PACKET_LENGTH];
static __pdata uint8_t last_sent_len;
static __pdata uint8_t last_recv_len;

// serial speed in 16usecs/byte
static __pdata uint16_t serial_rate;

// the length of a pending MAVLink packet, or zero if no MAVLink
// packet is expected
static __pdata uint8_t mav_pkt_len;

// the timer2_tick time when the MAVLink header was seen
static __pdata uint16_t mav_pkt_start_time;

// the number of timer2 ticks this packet should take on the serial link
static __pdata uint16_t mav_pkt_max_time;

static __pdata uint8_t mav_max_xmit;

// have we seen a mavlink packet?
bool seen_mavlink;
bool using_mavlink_10;

#define PACKET_RESEND_THRESHOLD 32

#define MAVLINK09_STX 85 // 'U'
#define MAVLINK10_STX 254

// check if a buffer looks like a MAVLink heartbeat packet - this
// is used to determine if we will inject RADIO status MAVLink
// messages into the serial stream for ground station and aircraft
// monitoring of link quality
static void check_heartbeat(__xdata uint8_t * __pdata buf)
{
	if (buf[0] == MAVLINK09_STX &&
	    buf[1] == 3 && buf[5] == 0) {
		// looks like a MAVLink 0.9 heartbeat
		using_mavlink_10 = false;
		seen_mavlink = true;
	} else if (buf[0] == MAVLINK10_STX &&
		   buf[1] == 9 && buf[5] == 0) {
		// looks like a MAVLink 1.0 heartbeat
		using_mavlink_10 = true;
		seen_mavlink = true;
	}
}

// return a complete MAVLink frame, possibly expanding
// to include other complete frames that fit in the max_xmit limit
static 
uint8_t packet_frame(__xdata uint8_t * __pdata buf)
{
	serial_read_buf(last_sent, mav_pkt_len);
	last_sent_len = mav_pkt_len;
	memcpy(buf, last_sent, last_sent_len);
	mav_pkt_len = 0;

	return last_sent_len;
}


// return the next packet to be sent
// In packet-only operation we want only to handle mavlink-inspired
// packets.  Bytes until a mavlink start character will be discarded
// (or later may be interpretted with special meaning), and exactly
// one mavlink-inspired packet will be pulled from the TX queue and
// dispatched, even if more than one would fit.
uint8_t
packet_get_next(register uint8_t max_xmit, __xdata uint8_t * __pdata buf)
{
	register uint16_t slen;

	// Send a packet from the TX buffer if there is one waiting.

	// Previously this was much more complex, now we only allow
	// popping exactly one packet from the tx buffer.
	// The packet format is <0xfe> <length> <sequence> <data>*length
	// Other data is silently ignored, but may in future be used to
	// perform other functions, such as query radio and link state.

	// if we have received something via serial see how
	// much of it we could fit in the transmit FIFO
	slen = serial_read_available();
	if (slen > max_xmit) {
		slen = max_xmit;
	}

	last_sent_len = 0;

	if (slen == 0) {
		// nothing available to send
		return 0;
	}

	while (slen > 0) {
		register uint8_t c = serial_peek();
		if (c != 0xfe) {
			// unrecognised command byte -- ignore
			serial_read();
		} else {
			// Packet start byte found -- see if we have the whole
			// packet ready

			// If there is only one byte, we can't determine the packet
			// length, so just return.
			if (slen == 1) return 0;
			// Get the length so that we can see if we have the whole packet
			// ready for sending
			mav_pkt_len = serial_peek2();
			// Check that we do indeed have the whole packet ready for sending
			if (slen<=1+1+mav_pkt_len) 
				// We are still waiting on bytes, so return now
				return 0;

			// At this point we know that we have the whole packet ready
			// and waiting.
			// We can now finally read the two header bytes and discard them.
			serial_read(); // 0xfe header byte
			serial_read(); // length byte

			// Now dispatch the packet.
			// This function reads the data from the tx buffer,
			// and populates the radio transmit buffer.
			// It takes mav_pkt_len as the number of bytes to send.
			return packet_frame(buf);
			}
		slen--;
	}
	
	// If no whole packet to send, then send nothing
	return 0;
}

// return true if the packet currently being sent
// is a resend
bool 
packet_is_resend(void)
{
	return last_sent_is_resend;
}

// force the last packet to be resent. Used when transmit fails
void
packet_force_resend(void)
{
	force_resend = true;
}

// set the maximum size of a packet
void
packet_set_max_xmit(uint8_t max)
{
	mav_max_xmit = max;
}

// set the serial speed in bytes/s
void
packet_set_serial_speed(uint16_t speed)
{
	// convert to 16usec/byte to match timer2_tick()
	serial_rate = (65536UL / speed) + 1;
}

// determine if a received packet is a duplicate
bool 
packet_is_duplicate(uint8_t len, __xdata uint8_t * __pdata buf, bool is_resend)
{
	if (!is_resend) {
		memcpy(last_received, buf, len);
		last_recv_len = len;
		last_recv_is_resend = false;
		return false;
	}
	if (last_recv_is_resend == false && 
	    len == last_recv_len &&
	    memcmp(last_received, buf, len) == 0) {
		last_recv_is_resend = false;
		return true;
	}
#if 0
	printf("RS(%u,%u)[", (unsigned)len, (unsigned)last_recv_len);
	serial_write_buf(last_received, last_recv_len);
	serial_write_buf(buf, len);
	printf("]\r\n");
#endif
	last_recv_is_resend = true;
	return false;
}
