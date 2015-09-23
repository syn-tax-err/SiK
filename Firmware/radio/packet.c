// -*- Mode: C; c-basic-offset: 8; -*-
//
// Copyright (c) 2012 Andrew Tridgell, All Rights Reserved
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

extern __xdata uint8_t pbuf[MAX_PACKET_LENGTH];

static __bit last_sent_is_resend;
static __bit last_sent_is_injected;
static __bit last_recv_is_resend;
static __bit force_resend;

static __xdata uint8_t last_received[MAX_PACKET_LENGTH];
static __xdata uint8_t last_sent[MAX_PACKET_LENGTH];
static __xdata uint8_t last_sent_len;
static __xdata uint8_t last_recv_len;

// serial speed in 16usecs/byte
static __pdata uint16_t serial_rate;

// the number of timer2 ticks this packet should take on the serial link
static __pdata uint16_t mav_pkt_max_time;

// true if we have a injected packet to send
static bool injected_packet;

#define PACKET_RESEND_THRESHOLD 32

uint8_t encryptReturn(__xdata uint8_t *buf_out, __xdata uint8_t *buf_in, uint8_t buf_in_len)
{
  // if no encryption or not supported fall back to copy
  memcpy(buf_out, buf_in, buf_in_len);
  return buf_in_len;
}

// return the next packet to be sent
uint8_t packet_get_next(register uint8_t max_xmit, __xdata uint8_t *buf)
{
	register uint16_t slen;

	if (injected_packet) {
		// send a previously injected packet
		slen = last_sent_len;

// sending these injected packets at full size doesn't
		// seem to work well ... though I don't really know why!
		if (max_xmit > 32) {
			max_xmit = 32;
		}
 
		if (max_xmit < slen) {
			// send as much as we can
			last_sent_len = slen - max_xmit;
			slen = encryptReturn(buf, last_sent, max_xmit);
			memcpy(last_sent, &last_sent[max_xmit], last_sent_len);
			last_sent_is_injected = true;
			return slen;
		}
		// send the rest
		injected_packet = false;
		last_sent_is_injected = true;
		return encryptReturn(buf, last_sent, last_sent_len);
	}
	last_sent_is_injected = false;

	slen = serial_read_available();
	if (force_resend ||
	    (feature_opportunistic_resend &&
	     last_sent_is_resend == false && 
	     last_sent_len != 0 && 
	     slen < PACKET_RESEND_THRESHOLD)) {
		if (max_xmit < last_sent_len) {
			return 0;
		}
		last_sent_is_resend = true;
		force_resend = false;
		return encryptReturn(buf, last_sent, last_sent_len);
	}

	last_sent_is_resend = false;

	// if we have received something via serial see how
	// much of it we could fit in the transmit FIFO
	if (slen > max_xmit) {
		slen = max_xmit;
	}

	last_sent_len = 0;

	if (slen == 0) {
		// nothing available to send
		return 0;
	}

	// simple (non-mavlink) framing
	if (slen > 0 && serial_read_buf(buf, slen)) {
		last_sent_len = slen;
		return encryptReturn(buf, last_sent, last_sent_len);
	}
	return 0;
}

// return true if the packet currently being sent
// is a resend
bool 
packet_is_resend(void)
{
	return last_sent_is_resend;
}

// return true if the packet currently being sent
// is an injected packet
bool 
packet_is_injected(void)
{
	return last_sent_is_injected;
}

// force the last packet to be resent. Used when transmit fails
void
packet_force_resend(void)
{
	force_resend = true;
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
packet_is_duplicate(uint8_t len, __xdata uint8_t *buf, bool is_resend)
{
	if (!is_resend) {
		memcpy(last_received, buf, len);
		last_recv_len = len;
		last_recv_is_resend = false;
		return false;
	}
	// We are now looking at a packet with the resend bit set
	if (last_recv_is_resend == false && 
	    len == last_recv_len &&
	    memcmp(last_received, buf, len) == 0) {
		last_recv_is_resend = false;
		return true;  // FIXME - this has no effect
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

// inject a packet to send when possible
void 
packet_inject(__xdata uint8_t *buf, __pdata uint8_t len)
{
	if (len > sizeof(last_sent)) {
		len = sizeof(last_sent);
	}
	memcpy(last_sent, buf, len);
	last_sent_len = len;
	last_sent_is_resend = false;
	injected_packet = true;
}
