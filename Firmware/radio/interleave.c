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
#else
#include <stdarg.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>
#endif

#define MAXBITS 512*8
#define BITSTEP 24 /* 3bytes*8bits */

#ifndef STANDALONE
__code const uint16_t steps[172]={
0, // n=0 bytes, no two bits are closer than 0 bits apart, total distance metric = 0 bits
7, // n=3 bytes, no two bits are closer than 6 bits apart, total distance metric = 140 bits
11, // n=6 bytes, no two bits are closer than 8 bits apart, total distance metric = 380 bits
11, // n=9 bytes, no two bits are closer than 12 bits apart, total distance metric = 852 bits
61, // n=12 bytes, no two bits are closer than 12 bits apart, total distance metric = 1144 bits
19, // n=15 bytes, no two bits are closer than 12 bits apart, total distance metric = 1436 bits
17, // n=18 bytes, no two bits are closer than 16 bits apart, total distance metric = 2290 bits
65, // n=21 bytes, no two bits are closer than 16 bits apart, total distance metric = 2674 bits
139, // n=24 bytes, no two bits are closer than 18 bits apart, total distance metric = 3440 bits
23, // n=27 bytes, no two bits are closer than 18 bits apart, total distance metric = 3876 bits
23, // n=30 bytes, no two bits are closer than 20 bits apart, total distance metric = 4784 bits
23, // n=33 bytes, no two bits are closer than 22 bits apart, total distance metric = 5788 bits
23, // n=36 bytes, no two bits are closer than 24 bits apart, total distance metric = 6888 bits
25, // n=39 bytes, no two bits are closer than 24 bits apart, total distance metric = 7466 bits
73, // n=42 bytes, no two bits are closer than 24 bits apart, total distance metric = 8044 bits
29, // n=45 bytes, no two bits are closer than 24 bits apart, total distance metric = 8622 bits
31, // n=48 bytes, no two bits are closer than 24 bits apart, total distance metric = 9200 bits
97, // n=51 bytes, no two bits are closer than 24 bits apart, total distance metric = 9778 bits
77, // n=54 bytes, no two bits are closer than 28 bits apart, total distance metric = 12070 bits
53, // n=57 bytes, no two bits are closer than 28 bits apart, total distance metric = 12742 bits
31, // n=60 bytes, no two bits are closer than 30 bits apart, total distance metric = 14372 bits
79, // n=63 bytes, no two bits are closer than 30 bits apart, total distance metric = 15092 bits
223, // n=66 bytes, no two bits are closer than 30 bits apart, total distance metric = 15812 bits
59, // n=69 bytes, no two bits are closer than 30 bits apart, total distance metric = 16536 bits
35, // n=72 bytes, no two bits are closer than 32 bits apart, total distance metric = 18404 bits
107, // n=75 bytes, no two bits are closer than 32 bits apart, total distance metric = 19172 bits
395, // n=78 bytes, no two bits are closer than 34 bits apart, total distance metric = 21184 bits
35, // n=81 bytes, no two bits are closer than 36 bits apart, total distance metric = 23292 bits
107, // n=84 bytes, no two bits are closer than 36 bits apart, total distance metric = 24156 bits
109, // n=87 bytes, no two bits are closer than 36 bits apart, total distance metric = 25022 bits
109, // n=90 bytes, no two bits are closer than 36 bits apart, total distance metric = 25888 bits
349, // n=93 bytes, no two bits are closer than 36 bits apart, total distance metric = 26752 bits
101, // n=96 bytes, no two bits are closer than 36 bits apart, total distance metric = 27618 bits
43, // n=99 bytes, no two bits are closer than 36 bits apart, total distance metric = 28484 bits
259, // n=102 bytes, no two bits are closer than 36 bits apart, total distance metric = 29348 bits
41, // n=105 bytes, no two bits are closer than 40 bits apart, total distance metric = 33562 bits
365, // n=108 bytes, no two bits are closer than 40 bits apart, total distance metric = 34522 bits
521, // n=111 bytes, no two bits are closer than 40 bits apart, total distance metric = 35482 bits
245, // n=114 bytes, no two bits are closer than 40 bits apart, total distance metric = 36442 bits
691, // n=117 bytes, no two bits are closer than 42 bits apart, total distance metric = 39272 bits
403, // n=120 bytes, no two bits are closer than 42 bits apart, total distance metric = 40280 bits
355, // n=123 bytes, no two bits are closer than 42 bits apart, total distance metric = 41288 bits
47, // n=126 bytes, no two bits are closer than 42 bits apart, total distance metric = 42300 bits
247, // n=129 bytes, no two bits are closer than 42 bits apart, total distance metric = 43308 bits
47, // n=132 bytes, no two bits are closer than 44 bits apart, total distance metric = 46424 bits
187, // n=135 bytes, no two bits are closer than 42 bits apart, total distance metric = 45324 bits
47, // n=138 bytes, no two bits are closer than 46 bits apart, total distance metric = 50740 bits
131, // n=141 bytes, no two bits are closer than 46 bits apart, total distance metric = 51844 bits
47, // n=144 bytes, no two bits are closer than 48 bits apart, total distance metric = 55248 bits
95, // n=147 bytes, no two bits are closer than 48 bits apart, total distance metric = 56400 bits
49, // n=150 bytes, no two bits are closer than 48 bits apart, total distance metric = 57554 bits
97, // n=153 bytes, no two bits are closer than 48 bits apart, total distance metric = 58706 bits
145, // n=156 bytes, no two bits are closer than 48 bits apart, total distance metric = 59860 bits
457, // n=159 bytes, no two bits are closer than 48 bits apart, total distance metric = 61012 bits
53, // n=162 bytes, no two bits are closer than 48 bits apart, total distance metric = 62166 bits
977, // n=165 bytes, no two bits are closer than 48 bits apart, total distance metric = 63318 bits
55, // n=168 bytes, no two bits are closer than 48 bits apart, total distance metric = 64472 bits
439, // n=171 bytes, no two bits are closer than 48 bits apart, total distance metric = 65624 bits
97, // n=174 bytes, no two bits are closer than 48 bits apart, total distance metric = 66778 bits
245, // n=177 bytes, no two bits are closer than 52 bits apart, total distance metric = 73582 bits
437, // n=180 bytes, no two bits are closer than 52 bits apart, total distance metric = 74830 bits
317, // n=183 bytes, no two bits are closer than 52 bits apart, total distance metric = 76078 bits
461, // n=186 bytes, no two bits are closer than 52 bits apart, total distance metric = 77326 bits
55, // n=189 bytes, no two bits are closer than 54 bits apart, total distance metric = 81596 bits
199, // n=192 bytes, no two bits are closer than 54 bits apart, total distance metric = 82892 bits
1207, // n=195 bytes, no two bits are closer than 54 bits apart, total distance metric = 84188 bits
103, // n=198 bytes, no two bits are closer than 54 bits apart, total distance metric = 85484 bits
103, // n=201 bytes, no two bits are closer than 54 bits apart, total distance metric = 86780 bits
395, // n=204 bytes, no two bits are closer than 54 bits apart, total distance metric = 88080 bits
515, // n=207 bytes, no two bits are closer than 54 bits apart, total distance metric = 89376 bits
59, // n=210 bytes, no two bits are closer than 56 bits apart, total distance metric = 94028 bits
329, // n=213 bytes, no two bits are closer than 56 bits apart, total distance metric = 95372 bits
137, // n=216 bytes, no two bits are closer than 56 bits apart, total distance metric = 96716 bits
227, // n=219 bytes, no two bits are closer than 58 bits apart, total distance metric = 101560 bits
491, // n=222 bytes, no two bits are closer than 58 bits apart, total distance metric = 102952 bits
59, // n=225 bytes, no two bits are closer than 60 bits apart, total distance metric = 107940 bits
269, // n=228 bytes, no two bits are closer than 60 bits apart, total distance metric = 109380 bits
299, // n=231 bytes, no two bits are closer than 60 bits apart, total distance metric = 110820 bits
301, // n=234 bytes, no two bits are closer than 60 bits apart, total distance metric = 112262 bits
229, // n=237 bytes, no two bits are closer than 60 bits apart, total distance metric = 113702 bits
181, // n=240 bytes, no two bits are closer than 60 bits apart, total distance metric = 115144 bits
133, // n=243 bytes, no two bits are closer than 60 bits apart, total distance metric = 116584 bits
253, // n=246 bytes, no two bits are closer than 60 bits apart, total distance metric = 118024 bits
205, // n=249 bytes, no two bits are closer than 60 bits apart, total distance metric = 119466 bits
277, // n=252 bytes, no two bits are closer than 60 bits apart, total distance metric = 120906 bits
67, // n=255 bytes, no two bits are closer than 60 bits apart, total distance metric = 122348 bits
667, // n=258 bytes, no two bits are closer than 60 bits apart, total distance metric = 123788 bits
283, // n=261 bytes, no two bits are closer than 60 bits apart, total distance metric = 125228 bits
65, // n=264 bytes, no two bits are closer than 64 bits apart, total distance metric = 135106 bits
245, // n=267 bytes, no two bits are closer than 64 bits apart, total distance metric = 136642 bits
1193, // n=270 bytes, no two bits are closer than 64 bits apart, total distance metric = 138178 bits
473, // n=273 bytes, no two bits are closer than 64 bits apart, total distance metric = 139714 bits
593, // n=276 bytes, no two bits are closer than 64 bits apart, total distance metric = 141250 bits
293, // n=279 bytes, no two bits are closer than 64 bits apart, total distance metric = 142786 bits
1675, // n=282 bytes, no two bits are closer than 66 bits apart, total distance metric = 148832 bits
1603, // n=285 bytes, no two bits are closer than 66 bits apart, total distance metric = 150416 bits
499, // n=288 bytes, no two bits are closer than 66 bits apart, total distance metric = 152000 bits
1651, // n=291 bytes, no two bits are closer than 66 bits apart, total distance metric = 153584 bits
1291, // n=294 bytes, no two bits are closer than 66 bits apart, total distance metric = 155168 bits
71, // n=297 bytes, no two bits are closer than 66 bits apart, total distance metric = 156756 bits
247, // n=300 bytes, no two bits are closer than 66 bits apart, total distance metric = 158340 bits
215, // n=303 bytes, no two bits are closer than 66 bits apart, total distance metric = 159924 bits
71, // n=306 bytes, no two bits are closer than 68 bits apart, total distance metric = 166400 bits
335, // n=309 bytes, no two bits are closer than 68 bits apart, total distance metric = 168032 bits
215, // n=312 bytes, no two bits are closer than 68 bits apart, total distance metric = 169664 bits
71, // n=315 bytes, no two bits are closer than 70 bits apart, total distance metric = 176332 bits
551, // n=318 bytes, no two bits are closer than 70 bits apart, total distance metric = 178012 bits
563, // n=321 bytes, no two bits are closer than 70 bits apart, total distance metric = 179692 bits
71, // n=324 bytes, no two bits are closer than 72 bits apart, total distance metric = 186552 bits
599, // n=327 bytes, no two bits are closer than 72 bits apart, total distance metric = 188280 bits
1031, // n=330 bytes, no two bits are closer than 70 bits apart, total distance metric = 184732 bits
73, // n=333 bytes, no two bits are closer than 72 bits apart, total distance metric = 191738 bits
1501, // n=336 bytes, no two bits are closer than 72 bits apart, total distance metric = 193466 bits
253, // n=339 bytes, no two bits are closer than 72 bits apart, total distance metric = 195194 bits
217, // n=342 bytes, no two bits are closer than 72 bits apart, total distance metric = 196924 bits
217, // n=345 bytes, no two bits are closer than 72 bits apart, total distance metric = 198650 bits
601, // n=348 bytes, no two bits are closer than 72 bits apart, total distance metric = 200380 bits
77, // n=351 bytes, no two bits are closer than 72 bits apart, total distance metric = 202110 bits
841, // n=354 bytes, no two bits are closer than 72 bits apart, total distance metric = 203838 bits
445, // n=357 bytes, no two bits are closer than 72 bits apart, total distance metric = 205566 bits
79, // n=360 bytes, no two bits are closer than 72 bits apart, total distance metric = 207296 bits
271, // n=363 bytes, no two bits are closer than 72 bits apart, total distance metric = 209024 bits
553, // n=366 bytes, no two bits are closer than 72 bits apart, total distance metric = 210752 bits
145, // n=369 bytes, no two bits are closer than 72 bits apart, total distance metric = 212482 bits
509, // n=372 bytes, no two bits are closer than 76 bits apart, total distance metric = 226102 bits
893, // n=375 bytes, no two bits are closer than 76 bits apart, total distance metric = 227926 bits
1181, // n=378 bytes, no two bits are closer than 76 bits apart, total distance metric = 229750 bits
173, // n=381 bytes, no two bits are closer than 76 bits apart, total distance metric = 231574 bits
149, // n=384 bytes, no two bits are closer than 76 bits apart, total distance metric = 233398 bits
605, // n=387 bytes, no two bits are closer than 76 bits apart, total distance metric = 235222 bits
79, // n=390 bytes, no two bits are closer than 78 bits apart, total distance metric = 243284 bits
511, // n=393 bytes, no two bits are closer than 78 bits apart, total distance metric = 245156 bits
427, // n=396 bytes, no two bits are closer than 78 bits apart, total distance metric = 247028 bits
283, // n=399 bytes, no two bits are closer than 78 bits apart, total distance metric = 248900 bits
175, // n=402 bytes, no two bits are closer than 78 bits apart, total distance metric = 250772 bits
439, // n=405 bytes, no two bits are closer than 78 bits apart, total distance metric = 252644 bits
151, // n=408 bytes, no two bits are closer than 78 bits apart, total distance metric = 254516 bits
803, // n=411 bytes, no two bits are closer than 78 bits apart, total distance metric = 256392 bits
155, // n=414 bytes, no two bits are closer than 78 bits apart, total distance metric = 258264 bits
1171, // n=417 bytes, no two bits are closer than 78 bits apart, total distance metric = 260136 bits
83, // n=420 bytes, no two bits are closer than 80 bits apart, total distance metric = 268724 bits
1337, // n=423 bytes, no two bits are closer than 80 bits apart, total distance metric = 270644 bits
635, // n=426 bytes, no two bits are closer than 80 bits apart, total distance metric = 272564 bits
587, // n=429 bytes, no two bits are closer than 80 bits apart, total distance metric = 274484 bits
2171, // n=432 bytes, no two bits are closer than 82 bits apart, total distance metric = 283312 bits
2123, // n=435 bytes, no two bits are closer than 82 bits apart, total distance metric = 285280 bits
299, // n=438 bytes, no two bits are closer than 82 bits apart, total distance metric = 287248 bits
83, // n=441 bytes, no two bits are closer than 84 bits apart, total distance metric = 296268 bits
827, // n=444 bytes, no two bits are closer than 84 bits apart, total distance metric = 298284 bits
317, // n=447 bytes, no two bits are closer than 84 bits apart, total distance metric = 300300 bits
283, // n=450 bytes, no two bits are closer than 84 bits apart, total distance metric = 302316 bits
589, // n=453 bytes, no two bits are closer than 84 bits apart, total distance metric = 304334 bits
373, // n=456 bytes, no two bits are closer than 84 bits apart, total distance metric = 306350 bits
805, // n=459 bytes, no two bits are closer than 84 bits apart, total distance metric = 308366 bits
421, // n=462 bytes, no two bits are closer than 84 bits apart, total distance metric = 310384 bits
727, // n=465 bytes, no two bits are closer than 84 bits apart, total distance metric = 312400 bits
823, // n=468 bytes, no two bits are closer than 84 bits apart, total distance metric = 314416 bits
1837, // n=471 bytes, no two bits are closer than 84 bits apart, total distance metric = 316432 bits
485, // n=474 bytes, no two bits are closer than 84 bits apart, total distance metric = 318450 bits
709, // n=477 bytes, no two bits are closer than 84 bits apart, total distance metric = 320466 bits
269, // n=480 bytes, no two bits are closer than 84 bits apart, total distance metric = 322482 bits
421, // n=483 bytes, no two bits are closer than 84 bits apart, total distance metric = 324500 bits
715, // n=486 bytes, no two bits are closer than 84 bits apart, total distance metric = 326516 bits
181, // n=489 bytes, no two bits are closer than 84 bits apart, total distance metric = 328532 bits
349, // n=492 bytes, no two bits are closer than 84 bits apart, total distance metric = 330548 bits
89, // n=495 bytes, no two bits are closer than 88 bits apart, total distance metric = 348394 bits
557, // n=498 bytes, no two bits are closer than 88 bits apart, total distance metric = 350506 bits
737, // n=501 bytes, no two bits are closer than 88 bits apart, total distance metric = 352618 bits
317, // n=504 bytes, no two bits are closer than 88 bits apart, total distance metric = 354730 bits
605, // n=507 bytes, no two bits are closer than 88 bits apart, total distance metric = 356842 bits
869, // n=510 bytes, no two bits are closer than 88 bits apart, total distance metric = 358954 bits
};

// The following are defined as macros to save RAM on the 8051 target, even though
// inline functions probably shouldn't consume RAM just by existing.

// We can use the pre-calculated optimised stepping patterns
#define bitnumber(n,bit) (((bit)*steps[n/3])%(n*8))
// ... or just spread bits evenly throughout space
//#define bitnumber(n,bit) (((bit)/48)+(n/6)*((bit)%48))
// It turns out that the optimised stepping patterns are MUCH better than evenly
// spreading the bits out through the encoded block.
// The optimal patterns allow for maximum burst errors of 11.6% on average (2.8 bits out of every 24), and upto 16.7% for some block lengths, i.e., ~4 out of every 24
// bits.
// In contrast, the even spreading can only do 7.2%, which is less than the 12.5%
// (3 bits in every 24) that it should be able to do.
// It is a mystery why both perform worse on average.  For now we will use the
// optimised stepping values.

#define interleave_getbit(n,in,bit) (((in[bitnumber(n,bit)>>3]>>(bitnumber(n,bit)&7))&1)?1:0)

#define interleave_setbit(n,in,bit, value) \
  { \
    if ((value)==0) in[(bitnumber(n,bit))>>3]&=~(1<<((bitnumber(n,bit))&7)); \
    else            in[(bitnumber(n,bit))>>3]|=  1<<((bitnumber(n,bit))&7);    \
  }

__xdata uint16_t interleave_data_size;

uint8_t interleave_getbyte(__xdata uint8_t * __pdata in,
			   __pdata uint16_t index)
{
  register uint8_t v=0;
  v|=interleave_getbit(interleave_data_size,in,index*8+0);
  v|=interleave_getbit(interleave_data_size,in,index*8+1)<<1;
  v|=interleave_getbit(interleave_data_size,in,index*8+2)<<2;
  v|=interleave_getbit(interleave_data_size,in,index*8+3)<<3;
  v|=interleave_getbit(interleave_data_size,in,index*8+4)<<4;
  v|=interleave_getbit(interleave_data_size,in,index*8+5)<<5;
  v|=interleave_getbit(interleave_data_size,in,index*8+6)<<6;
  v|=interleave_getbit(interleave_data_size,in,index*8+7)<<7;
  return v;
}

void interleave_setbyte(__xdata uint8_t * __pdata in,
			__pdata uint16_t index, uint8_t __pdata value)
{
  interleave_setbit(interleave_data_size,in,index*8+0,(value>>0)&1);
  interleave_setbit(interleave_data_size,in,index*8+1,(value>>1)&1);
  interleave_setbit(interleave_data_size,in,index*8+2,(value>>2)&1);
  interleave_setbit(interleave_data_size,in,index*8+3,(value>>3)&1);
  interleave_setbit(interleave_data_size,in,index*8+4,(value>>4)&1);
  interleave_setbit(interleave_data_size,in,index*8+5,(value>>5)&1);
  interleave_setbit(interleave_data_size,in,index*8+6,(value>>6)&1);
  interleave_setbit(interleave_data_size,in,index*8+7,(value>>7)&1);
}

#else

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
  for(n=0;n<MAXBITS;n+=BITSTEP) {
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
