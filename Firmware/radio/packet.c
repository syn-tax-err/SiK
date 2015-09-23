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

#define MAVLINK_MSG_ID_X_SERVAL_SETUPRADIO 210
#define MAVLINK_MSG_ID_X_SERVAL_RESULT 211
// use 'ME' for Mesh Extender
#define RADIO_SOURCE_SYSTEM 'M'
#define RADIO_SOURCE_COMPONENT 'E'

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

// the length of a pending MAVLink packet, or zero if no MAVLink
// packet is expected
static __pdata uint8_t mav_pkt_len;

// the timer2_tick time when the MAVLink header was seen
static __pdata uint16_t mav_pkt_start_time;

// the number of timer2 ticks this packet should take on the serial link
static __pdata uint16_t mav_pkt_max_time;

static __pdata uint8_t mav_max_xmit;

// true if we have a injected packet to send
static bool injected_packet;

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
	} else if (buf[0] == MAVLINK10_STX &&
		   // 226 is arbitrarily chosen unallocated MAVLink message ID
		   // used to indicate this set radio parameters command
		   buf[1] == 26 && buf[5] == MAVLINK_MSG_ID_X_SERVAL_SETUPRADIO &&
		   // some magic bytes in other MAVLink fields
		   buf[2] == 0xBE && buf[3] == 0xEF && buf[4] == 0xEE ) {
		// looks like a Serval Mesh Extender Radio Configure packet
		// This takes a while to run, and will confuse the TDM, 
		// but that's okay, because it only needs to happen once.
		bool reject=false;
		uint32_t value;
		// NETID low byte		
		// NETID high byte		
		value=(buf[6]<<8)||buf[7];
		if (param_set(PARAM_NETID,value)) reject=true;
		// TX power
		if (param_set(PARAM_TXPOWER,buf[8])) reject=true;
		// Duty cycle
		if (param_set(PARAM_DUTY_CYCLE,buf[9])) reject=true;		
		// ECC enable
		if (param_set(PARAM_ECC,buf[10])) reject=true;
		// Air speed
		if (param_set(PARAM_AIR_SPEED,buf[11])) reject=true;
		// UART speed
		if (param_set(PARAM_SERIAL_SPEED,buf[12])) reject=true;
		// Opportunistic resend
		if (param_set(PARAM_OPPRESEND,buf[13])) reject=true;
		// unused (was Number of channels)
		// if (param_set(PARAM_NUM_CHANNELS,buf[14])) reject=true;
		// MAVLink enable
		if (param_set(PARAM_MAVLINK,buf[15])) reject=true;
		// LBT RSSI
		if (param_set(PARAM_LBT_RSSI,buf[16])) reject=true;
		// Manchester encoding
		if (param_set(PARAM_MANCHESTER,buf[17])) reject=true;
		// channel frequency
		value=buf[18]; value=value<<8;
		value=buf[19]; value=value<<8;
		value=buf[20]; value=value<<8;
		value=buf[21]; 
		if (param_set(PARAM_FREQ,value)) reject=true;
		// unused (was upper frequency)
		value=buf[22]; value=value<<8;
		value=buf[23]; value=value<<8;
		value=buf[24]; value=value<<8;
		value=buf[25]; 

		if (!reject) param_save();

		// Send response
		pbuf[0] = using_mavlink_10?254:'U';
		pbuf[1] = 2;
		pbuf[2] = 0xff;
		pbuf[3] = RADIO_SOURCE_SYSTEM;
		pbuf[4] = RADIO_SOURCE_COMPONENT;
		pbuf[5] = MAVLINK_MSG_ID_X_SERVAL_RESULT;
		
		pbuf[6]=MAVLINK_MSG_ID_X_SERVAL_SETUPRADIO;
		pbuf[7]=reject?0xff:0x00;
		
		if (serial_write_space() < 6+2+2) {
			// don't cause an overflow
			return;
		}
		
		serial_write_buf(pbuf, 6+2+2);			

	}
}

// return a complete MAVLink frame, possibly expanding
// to include other complete frames that fit in the max_xmit limit
static 
uint8_t mavlink_frame(uint8_t max_xmit, __xdata uint8_t * __pdata buf)
{
	__data uint16_t slen;

	serial_read_buf(last_sent, mav_pkt_len);
	last_sent_len = mav_pkt_len;
	memcpy(buf, last_sent, last_sent_len);
	mav_pkt_len = 0;

	check_heartbeat(buf);

	slen = serial_read_available();

	// see if we have more complete MAVLink frames in the serial
	// buffer that we can fit in this packet
	while (slen >= 8) {
		register uint8_t c = serial_peek();
		if (c != MAVLINK09_STX && c != MAVLINK10_STX) {
			// its not a MAVLink packet
			return last_sent_len;			
		}
		c = serial_peek2();
		if (c >= 255 - 8 || 
		    c+8 > max_xmit - last_sent_len) {
			// it won't fit
			break;
		}
		if (c+8 > slen) {
			// we don't have the full MAVLink packet in
			// the serial buffer
			break;
		}

		c += 8;

		// we can add another MAVLink frame to the packet
		serial_read_buf(&last_sent[last_sent_len], c);
		memcpy(&buf[last_sent_len], &last_sent[last_sent_len], c);

		check_heartbeat(buf+last_sent_len);

		last_sent_len += c;
		slen -= c;
	}

	return last_sent_len;
}

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

	if (!feature_mavlink_framing) {
		// simple framing
		if (slen > 0 && serial_read_buf(buf, slen)) {
			last_sent_len = slen;
			return encryptReturn(buf, last_sent, last_sent_len);
		}
		return 0;
	}

	// try to align packet boundaries with MAVLink packets

	if (mav_pkt_len == 1) {
		// we're waiting for the MAVLink length byte
		if (slen == 1) {
			if ((uint16_t)(timer2_tick() - mav_pkt_start_time) > mav_pkt_max_time) {
				// we didn't get the length byte in time
				last_sent[last_sent_len++] = serial_read();	
				mav_pkt_len = 0;
				return encryptReturn(buf, last_sent, last_sent_len);
			}
			// still waiting ....
			return 0;
		}
		// we have more than one byte, use normal packet frame
		// detection below
		mav_pkt_len = 0;
	}


	if (mav_pkt_len != 0) {
		if (slen < mav_pkt_len) {
			if ((uint16_t)(timer2_tick() - mav_pkt_start_time) > mav_pkt_max_time) {
				// timeout waiting for the rest of
				// it. Send what we have now.
				serial_read_buf(last_sent, slen);
				last_sent_len = slen;
				memcpy(buf, last_sent, last_sent_len);
				mav_pkt_len = 0;
				return last_sent_len;
			}
			// leave it in the serial buffer till we have the
			// whole MAVLink packet			
			return 0;
		}
		
		// the whole of the MAVLink packet is available
		return mavlink_frame(max_xmit, buf);
	}
		
	// We are now looking for a new packet (mav_pkt_len == 0)
	while (slen > 0) {
		register uint8_t c = serial_peek();
		if (c == MAVLINK09_STX || c == MAVLINK10_STX) {
			if (slen == 1) {
				// we got a bare MAVLink header byte
				if (last_sent_len == 0) {
					// wait for the next byte to
					// give us the length
					mav_pkt_len = 1;
					mav_pkt_start_time = timer2_tick();
					mav_pkt_max_time = serial_rate;
					return 0;
				}
				break;
			}
			mav_pkt_len = serial_peek2();
			if (mav_pkt_len >= 255-8 ||
			    mav_pkt_len+8 > mav_max_xmit) {
				// its too big for us to cope with
				mav_pkt_len = 0;
				last_sent[last_sent_len++] = serial_read();
				slen--;				
				continue;
			}

			// the length byte doesn't include
			// the header or CRC
			mav_pkt_len += 8;
			
			if (last_sent_len != 0) {
				// send what we've got so far,
				// and send the MAVLink payload
				// in the next packet
				mav_pkt_start_time = timer2_tick();
				mav_pkt_max_time = mav_pkt_len * serial_rate;
				return encryptReturn(buf, last_sent, last_sent_len);
			} else if (mav_pkt_len > slen) {
				// the whole MAVLink packet isn't in
				// the serial buffer yet. 
				mav_pkt_start_time = timer2_tick();
				mav_pkt_max_time = mav_pkt_len * serial_rate;
				return 0;					
			} else {
				// the whole packet is there
				// and ready to be read
				// TODO FIX THIS FOR ENCRYPT
				return mavlink_frame(max_xmit, buf);
			}
		} else {
			last_sent[last_sent_len++] = serial_read();
			slen--;
		}
	}
	return encryptReturn(buf, last_sent, last_sent_len);
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
