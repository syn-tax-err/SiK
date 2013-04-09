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
/// @file	golay23.c
///
/// golay 23/12 error correction encoding and decoding
///

#include <stdarg.h>
#ifndef INTERLEAVE_TEST
#include "radio.h"
#include "crc.h"
#endif
#include "golay23.h"

// intermediate arrays for encodeing/decoding. Using these
// saves some interal memory that would otherwise be needed
// for pointers
static __pdata uint8_t g3[3], g6[6];

#define GOLAY_POLY 0xc75UL

static const __code uint32_t shift_table[12] = {
	GOLAY_POLY<<0,
	GOLAY_POLY<<1,
	GOLAY_POLY<<2,
	GOLAY_POLY<<3,
	GOLAY_POLY<<4,
	GOLAY_POLY<<5,
	GOLAY_POLY<<6,
	GOLAY_POLY<<7,
	GOLAY_POLY<<8,
	GOLAY_POLY<<9,
	GOLAY_POLY<<10,
	GOLAY_POLY<<11,
};

// calculate the golay syndrome value
static uint16_t 
golay_syndrome(__data uint32_t codeword)
{
	__data uint32_t shift = (1UL<<22);
	__data uint8_t shiftcount = 11;

	while (codeword >= (1UL<<11)) {
		while ((shift & codeword) == 0) {
			shift >>= 1;
			shiftcount--;
		}
		codeword ^= shift_table[shiftcount];
	}
	return codeword;
}


// encode 3 bytes data into 6 bytes of coded data
// input is in g3[], output in g6[]
// This uses complete lookup tables.
static void 
golay_encode24(void)
{
	__pdata uint16_t v;
	__pdata uint32_t codeword;

	v = g3[0] | ((uint16_t)g3[1]&0xF)<<8;
	codeword = golay23_encode[v];
	g6[0] = codeword & 0xFF;
	g6[1] = (codeword >> 8) & 0xFF;
	g6[2] = (codeword >> 16) & 0xFF;

	v = g3[2] | ((uint16_t)g3[1]&0xF0)<<4;
	codeword = golay23_encode[v];
	g6[3] = codeword & 0xFF;
	g6[4] = (codeword >> 8) & 0xFF;
	g6[5] = (codeword >> 16) & 0xFF;
}

void interleave_setbyte(__xdata uint8_t * __pdata in,
			__pdata uint16_t index, uint8_t __pdata value);
uint8_t interleave_getbyte(__xdata uint8_t * __pdata in,
			   __pdata uint16_t index);
extern __xdata uint16_t interleave_data_size;

// encode n bytes of data into 2n coded bytes. n must be a multiple 3
// encoding takes about 6 microseconds per input byte

int show(char *m,int n, uint8_t *b);

__xdata uint8_t offset_start;
__xdata uint8_t offset_end;
void 
golay_encode_portion(__pdata uint8_t en, __xdata uint8_t * __pdata in_piece, __xdata uint8_t * __pdata out)
{
	uint8_t i;
	interleave_data_size=en;
	for(i=offset_start;i+2<=offset_end;i+=3) {
		g3[0] = in_piece[i+0-offset_start]; 
		g3[1] = in_piece[i+1-offset_start]; 
		g3[2] = in_piece[i+2-offset_start];
		golay_encode24();
		if (param_get(PARAM_ECC)==1) {
			// Non-interleaved output
			out[i*2+0] = g6[0]; out[i*2+1] = g6[1]; out[i*2+2] = g6[2]; 
			out[i*2+3] = g6[3]; out[i*2+4] = g6[4]; out[i*2+5] = g6[5];
		} else {
			// Interleaved output to strengthen against burst errors
			interleave_setbyte(out,i*2+0,g6[0]);
			interleave_setbyte(out,i*2+1,g6[1]);
			interleave_setbyte(out,i*2+2,g6[2]);
			interleave_setbyte(out,i*2+3,g6[3]);
			interleave_setbyte(out,i*2+4,g6[4]);
			interleave_setbyte(out,i*2+5,g6[5]);
		}
	}
#ifdef INTERLEAVE_TEST
	if (verbose) {
		printf("After encoding bytes [%d..%d] (en=%d)\n",
		       offset_start,offset_end,en);
		show("bytes being encoded",offset_end-offset_start+1,in_piece);
		show("buffer",en,out);
	}
#endif
	
}

void
golay_encode(__pdata uint8_t n, __xdata uint8_t * __pdata in, __xdata uint8_t * __pdata out)
{
	uint8_t i;
	interleave_data_size=n*2;
	for(i=0;i!=n;i+=3) {
		g3[0] = in[i+0]; g3[1] = in[i+1]; g3[2] = in[i+2];
		golay_encode24();
		if (param_get(PARAM_ECC)==1) {
			// Non-interleaved output
			out[i*2+0] = g6[0]; out[i*2+1] = g6[1]; out[i*2+2] = g6[2]; 
			out[i*2+3] = g6[3]; out[i*2+4] = g6[4]; out[i*2+5] = g6[5];
		} else {
			// Interleaved output to strengthen against burst errors
			interleave_setbyte(out,i*2+0,g6[0]);
			interleave_setbyte(out,i*2+1,g6[1]);
			interleave_setbyte(out,i*2+2,g6[2]);
			interleave_setbyte(out,i*2+3,g6[3]);
			interleave_setbyte(out,i*2+4,g6[4]);
			interleave_setbyte(out,i*2+5,g6[5]);
		}
	}
}

// decode 6 bytes of coded data into 3 bytes of original data
// input is in g6[], output in g3[]
// returns the number of words corrected (0, 1 or 2)
static uint8_t 
golay_decode24(void)
{
	__data uint16_t v, v0;
	__data uint32_t codeword;
	__pdata uint8_t errcount = 0;

	codeword = g6[0] | (((uint16_t)g6[1])<<8) | (((uint32_t)(g6[2]&0x7F))<<16);
	v0 = codeword >> 11;
	v = golay_syndrome(codeword);
	codeword ^= golay23_decode[v];
	v = codeword >> 11;
	if (v != v0) {
		errcount++;
	}

	g3[0] = v & 0xFF;
	g3[1] = (v >> 8);

	codeword = g6[3] | (((uint16_t)g6[4])<<8) | (((uint32_t)(g6[5]&0x7F))<<16);
	v0 = codeword >> 11;
	v = golay_syndrome(codeword);
	codeword ^= golay23_decode[v];
	v = codeword >> 11;
	if (v != v0) {
		errcount++;
	}

	g3[1] |= ((v >> 4)&0xF0);
	g3[2] = v & 0xFF;
	return errcount;
}

// decode n bytes of coded data into n/2 bytes of original data
// n must be a multiple of 6
// decoding takes about 20 microseconds per input byte
// the number of 12 bit words that required correction is returned
uint8_t 
golay_decode(__pdata uint8_t n, __xdata uint8_t * __pdata in, __xdata uint8_t * __pdata out)
{
	__pdata uint8_t errcount = 0;
	uint8_t i;
	interleave_data_size=n;
	for(i=0;i<n;i+=6) {
		if (param_get(PARAM_ECC)==1) {
			g6[0] = in[0]; g6[1] = in[1]; g6[2] = in[2];
			g6[3] = in[3]; g6[4] = in[4]; g6[5] = in[5];
			in += 6;
		} else {
			g6[0]= interleave_getbyte(in,i+0);
			g6[1]= interleave_getbyte(in,i+1);
			g6[2]= interleave_getbyte(in,i+2);
			g6[3]= interleave_getbyte(in,i+3);
			g6[4]= interleave_getbyte(in,i+4);
			g6[5]= interleave_getbyte(in,i+5);
		}
		errcount += golay_decode24();
		out[0] = g3[0]; out[1] = g3[1]; out[2] = g3[2];
		out += 3;
	}
	return errcount;
}

void
golay_encode_packet(uint8_t length, __xdata uint8_t * __pdata buf)
{
	__pdata uint16_t crc;
	__xdata uint8_t gin[3];
	__xdata uint8_t elen, rlen;

	if (length > (sizeof(radio_buffer)/2)-6) {
		debug("golay packet size %u\n", (unsigned)length);
		panic("oversized golay packet");		
	}

	// rounded length
	rlen = ((length+2)/3)*3;

	// encoded length
	elen = (rlen+6)*2;

	// start of packet is network ID and packet length
	gin[0] = netid[0];
	gin[1] = netid[1];
	gin[2] = length;
	
	// golay encode the header
	offset_start=0; offset_end=2;
	golay_encode_portion(elen,gin, radio_buffer);
	
	// next add a CRC, we round to 3 bytes for simplicity, adding 
	// another copy of the length in the spare byte
	crc = crc16(length, buf);
	gin[0] = crc&0xFF;
	gin[1] = crc>>8;
	gin[2] = length;
	
	// golay encode the CRC
	offset_start=3; offset_end=5;
	golay_encode_portion(elen,gin, radio_buffer);
	
	// encode the rest of the payload
	if (rlen>6) {
		offset_start=6; offset_end=rlen-1;
		golay_encode_portion(elen,buf, radio_buffer);
	}
	radio_buffer_count=elen;
}

bool 
golay_decode_packet(uint8_t *length,__xdata uint8_t * __pdata buf,__xdata uint8_t elen)
{
	// Packet is already in radio_interleave_buffer, and length
	// is in receive_packet_length

	__xdata uint16_t crc1, crc2;
	__xdata uint8_t errcount = 0;
	__xdata uint8_t l;

	if (elen < 12 || (elen%6) != 0) {
		// not a valid length
		if (at_testmode&AT_TEST_FEC&&(param_get(PARAM_ECC)>0))
			printf("rx len invalid %u\n", (unsigned)elen);
		goto failed;
	}

	// decode the entire packet (necessary due to interleaving,
	// even though we might waste some processor power decoding
	// packets that are not for us).
	errcount = golay_decode(elen,radio_interleave_buffer,buf);
#ifdef INTERLEAVE_TEST
	if (verbose) {
		printf("elen=%d\n",elen);
		show("radio_interleave_buffer",elen,radio_interleave_buffer);
		show("buf",elen/2,buf);
	}
#endif
	// buf now contains the decoded packet, including headers.

	// Check netid
	if (buf[0] != netid[0] ||
	    buf[1] != netid[1]) {
		// its not for our network ID 		
		if (at_testmode&AT_TEST_FEC&&(param_get(PARAM_ECC)>0))		
			printf("netid %x %x is not us (len=%u).\n",
			       (unsigned)buf[0],
			       (unsigned)buf[1],
			       (unsigned)buf[2]);
		goto failed;
	}

	if (6*((buf[2]+2)/3+2) != elen) {
		if (at_testmode&AT_TEST_FEC&&(param_get(PARAM_ECC)>0))		
			printf("rx len mismatch1 %u %u\n",
			       (unsigned)buf[2],
			       (unsigned)elen);	
		goto failed;
	}

	// extract the sender CRC
	crc1 = buf[3+0] | (((uint16_t)buf[3+1])<<8);

	*length = buf[3+2];
	
	// Copy buffer down over headers ready for calculating CRC
	for(l=0;l<buf[3+2];l++) buf[l]=buf[l+6];
	//	memcpy(radio_interleave_buffer,&buf[6],*length);
       
	l=buf[3+2];
	crc2 = crc16(l, buf);
	
	if (crc1 != crc2) {
		if (at_testmode&AT_TEST_FEC&&(param_get(PARAM_ECC)>0)) {
			printf("CRC error");
			printf(": crc in header=%x.%x", 
			       buf[3+1],buf[3+0]);
			printf(" crc of data=%x",
			       (unsigned)crc2);
			printf(" len=%u",l);
			printf(" [%x %x]\n",
			       (unsigned)buf[0],
			       (unsigned)buf[1]);
		}
		goto failed;
	}

	if (errcount != 0) {
		if ((uint16_t)(0xFFFF - errcount) > errors.corrected_errors) {
			errors.corrected_errors += errcount;
		} else {
			errors.corrected_errors = 0xFFFF;
		}
		if (errors.corrected_packets != 0xFFFF) {
			errors.corrected_packets++;
		}
	}

	if (at_testmode&AT_TEST_FEC&&(param_get(PARAM_ECC)>0)) 
		printf("Received OK packet (len=%u)\n",l);
	return true;

 failed:
	if (errors.rx_errors != 0xFFFF) {
		errors.rx_errors++;
	}
	return false;
}
