// -*- Mode: C; c-basic-offset: 8; -*-
//
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
/// @file	interleave.c
///
/// interleave and de-interleave packets before golay encoding to
/// improve performance of golay error detection/correction in the
/// face of burst errors.
///
/// As the golay coder can correct 3 in every 12 bits, a byte-level
/// interleaver is probably sufficient for now. 

#ifdef STANDALONE
#include <stdio.h>
#define __pdata
#define __xdata
#define uint8_t unsigned char
#define uint16_t unsigned short
#endif

#define MAXBITS 256*8
#define BITSTEP 24 /* 3bytes*8bits */

#include "steps.c"

static uint8_t interleave_getbit(__pdata uint8_t n, __xdata uint8_t * __pdata in,
				 __pdata uint8_t bit)
{
  return (in[((bit*steps[n/3])%n)>>3]
	  >>(((bit*steps[n/3])%n)&7))?1:0;
}

static void interleave_setbit(__pdata uint8_t n, __xdata uint8_t * __pdata in,
			      __pdata uint8_t bit, __pdata uint8_t value)
{
  uint8_t byte=in[((bit*steps[n/3])%n)>>3];
  bit=1<<(((bit*steps[n/3])%n)&7);
  if (value) byte|=bit; else byte&=~bit;
}

uint8_t interleave_getbyte(__pdata uint8_t n, __xdata uint8_t * __pdata in,
			   __pdata uint8_t index)
{
  uint8_t v=0;
  v|=interleave_getbit(n,in,index*8+0);
  v|=interleave_getbit(n,in,index*8+1)<<1;
  v|=interleave_getbit(n,in,index*8+2)<<2;
  v|=interleave_getbit(n,in,index*8+3)<<3;
  v|=interleave_getbit(n,in,index*8+4)<<4;
  v|=interleave_getbit(n,in,index*8+5)<<5;
  v|=interleave_getbit(n,in,index*8+6)<<6;
  v|=interleave_getbit(n,in,index*8+7)<<7;
  return v;
}

void interleave_setbyte(__pdata uint8_t n, __xdata uint8_t * __pdata in,
			__pdata uint8_t index, uint8_t __pdata value)
{
  interleave_setbit(n,in,(index<<3)+0,(value>>0)&1);
  interleave_setbit(n,in,(index<<3)+1,(value>>1)&1);
  interleave_setbit(n,in,(index<<3)+2,(value>>2)&1);
  interleave_setbit(n,in,(index<<3)+3,(value>>3)&1);
  interleave_setbit(n,in,(index<<3)+4,(value>>4)&1);
  interleave_setbit(n,in,(index<<3)+5,(value>>5)&1);
  interleave_setbit(n,in,(index<<3)+6,(value>>6)&1);
  interleave_setbit(n,in,(index<<3)+7,(value>>7)&1);
}

#ifdef STANDALONE

int prefill(int *b)
{
  int i;
  for(i=0;i<MAXBITS;i++) b[i]=i;
  return 0;
}

int show(int n,int *b)
{
  int i;
  for(i=0;i<n;i++) {
    if (!(i&0xf)) printf("\n%02x:",i);
    printf(" %02x",b[i]);
  }
  printf("\n");
  return 0;
}

int main(int argc,char **argv)
{
  int b[MAXBITS];
  int c[MAXBITS];
  int n,i,j;
  unsigned short step=0;
  printf("uint16_t steps[%d]={\n",MAXBITS/BITSTEP+1);
  for(n=1;n<MAXBITS;n+=BITSTEP) {
    int bestmindiff=-1;
    int besttotaldiff=-1;
    int beststep=-1;
    for(step=0;step<MAXBITS;step++) {
      prefill(b);
      bzero(&c[0],MAXBITS*sizeof(int));
      for(i=0;i<n;i++) {
	b[i]=(i*step)%n;
	c[(i*step)%n]++;
      }
      for(i=0;i<n;i++) if (c[i]!=1) {
	  break;
	}
      if (i==n) {
	int mindiff=n;
	int totaldiff=0;
	for(i=0;i<n-1;i++) {
	  int diff=MAXBITS;
	  for(j=0;j<i;j++) {
	    int bdiff=b[i]-b[j]; if (bdiff<0) bdiff=-bdiff;
	    bdiff+=(i-j);	    
	    if (bdiff<diff) diff=bdiff;
	  }
	  for(j=i+1;j<n;j++) {
	    int bdiff=b[i]-b[j]; if (bdiff<0) bdiff=-bdiff;
	    bdiff+=(j-i);	    
	    if (bdiff<diff) diff=bdiff;
	  }

	  if (diff<0) diff=-diff;
	  if (diff<mindiff) mindiff=diff;
	  if (mindiff<bestmindiff) break;
	  totaldiff+=diff;
	}
	if (mindiff>bestmindiff
	    ||(mindiff==bestmindiff&&totaldiff>besttotaldiff)) {
	  beststep=step;
	  bestmindiff=mindiff;
	  besttotaldiff=totaldiff;
	  //	  printf("n=%d, step=%d, mindiff=%d, totaldiff=%d\n",
	  //	 n,step,mindiff,totaldiff);
	}
      }
    }
    printf("%d%s // n=%d bytes, no two bits are closer than %d bits apart, total distance metric = %d bits\n",
	   beststep,(n!=MAXBITS/BITSTEP-1)?",":"",
	   n/8,bestmindiff,besttotaldiff);
  }
  printf("};\n");
}

#endif
