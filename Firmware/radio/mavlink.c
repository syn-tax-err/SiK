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
/// @file	mavlink.c
///
/// mavlink reporting code
///

#include <stdarg.h>
#include "radio.h"
#include "packet.h"
#include "timer.h"

extern __xdata uint8_t pbuf[MAX_PACKET_LENGTH];
static __pdata uint8_t seqnum;

#define MAVLINK_MSG_ID_RADIO 166
#define MAVLINK_RADIO_CRC_EXTRA 21

// new RADIO_STATUS common message
#define MAVLINK_MSG_ID_RADIO_STATUS 109
#define MAVLINK_RADIO_STATUS_CRC_EXTRA 185

// use '3D' for 3DRadio
#define RADIO_SOURCE_SYSTEM '3'
#define RADIO_SOURCE_COMPONENT 'D'


static void swap_bytes(__pdata uint8_t ofs, __pdata uint8_t len)
{
	register uint8_t i;
	for (i=ofs; i<ofs+len; i+=2) {
		register uint8_t tmp = pbuf[i];
		pbuf[i] = pbuf[i+1];
		pbuf[i+1] = tmp;
	}
}

