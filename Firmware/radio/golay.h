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
/// @file	golay23.h
///
/// golay 23/12 error correction encoding and decoding
///

// Collect timing stats for golay decoding
extern __xdata uint16_t golay_decode_start_time;
extern __xdata uint16_t golay_decode_end_time;
extern __xdata uint8_t golay_decode_time_bytes;

/// encode n bytes of data into 2n coded bytes. n must be a multiple 3
extern void golay_encode(__pdata uint8_t n, __xdata uint8_t * __pdata in, __xdata uint8_t * __pdata out);

/// As above, but encode only a portion of a complete buffer.
/// This is needed for interleaved golay encoding, where it is not possible to
/// just append the pieces together.  The range of the output buffer to be encoded
/// is specified with offset_start and offset_end.
extern __xdata uint8_t offset_start;
extern __xdata uint8_t offset_end;
extern void golay_encode_portion(__pdata uint8_t en, __xdata uint8_t * __pdata in_piece, __xdata uint8_t * __pdata out);

/// decode n bytes of coded data into n/2 bytes of original data
/// n must be a multiple of 6
extern uint8_t golay_decode(__pdata uint8_t n, __xdata uint8_t * __pdata in, __xdata uint8_t * __pdata out);

// Prepare a packet for transmission
extern void golay_encode_packet(uint8_t length, __xdata uint8_t * __pdata buf);

// The reverse of the above: take such a prepared packet and decode it.
extern bool golay_decode_packet(uint8_t *length,__xdata uint8_t * __pdata buf,__xdata uint8_t elen);
