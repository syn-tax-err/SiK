#include <stdarg.h>
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

#define INTERLEAVE_TEST
#define __code
#define __data
#define __pdata
#define __xdata
#define PARAM_ECC 1

uint8_t radio_buffer[252];
uint8_t radio_buffer_count;
uint8_t netid[2]={0xaa,0x55};
#define debug(fmt, args...)

int verbose=0;

int feature_golay=1;
int feature_golay_interleaving=1;
int interleave=1;
int param_get(int param)
{
  // ECC_PARAM returns "Golay + interlacing"
  if (interleave)
    return 2;
  else return 1;
}
int setInterleaveP(int yesno)
{
  interleave=yesno;
  feature_golay_interleaving=yesno;
}

#define AT_TEST_FEC 4
int at_testmode=0;

uint8_t radio_interleave_buffer[256];

struct error_counts {
	uint16_t rx_errors;		///< count of packet receive errors
	uint16_t tx_errors;		///< count of packet transmit errors
	uint16_t serial_tx_overflow;    ///< count of serial transmit overflows
	uint16_t serial_rx_overflow;    ///< count of serial receive overflows
	uint16_t corrected_errors;      ///< count of words corrected by golay code
	uint16_t corrected_packets;     ///< count of packets corrected by golay code
};
struct error_counts errors;

#include "radio/crc.c"
#include "radio/interleave.c"
#include "radio/golay.c"

int show(char *msg,int n,unsigned char *b)
{
  int i;
  printf("%s:\n",msg);
  for(i=0;i<n;i++) {
    if (!(i&0xf)) printf("\n%02x:",i);
    printf(" %02x",b[i]);
  }
  printf("\n");
  return 0;
}

int prefill(unsigned char *b)
{
  int i;
  for(i=0;i<256;i++) b[i]=i;
  return 0;
}

int countones(int n,unsigned char *b)
{
  int j,i=0;
  int count=0;
  for(i=0;i<n;i++) {
    for(j=0;j<8;j++)
      if (b[i]&(1<<j)) count++;
  }
  return count;
}

int showbitpattern(int n,int byte_low,int byte_high,int burst_low,int burst_high)
{
  printf("Interleaved bit pattern: (n=%d, range=[%d,%d)\n",n,byte_low,byte_high);
  int i,j;
  int count=0;
  for(i=byte_low*8;i<byte_high*8;i+=8)
    {
      int affected=0;
      for(j=i+7;j>=i;j--)
	{
	  int b=bitnumber(n,j);
	  if (b>=burst_low&&b<=burst_high)
	    affected=1;
	}
      if (affected) {
	printf("byte 0x%02x : ",i/8);
	for(j=i+7;j>=i;j--)
	  {
	    int b=bitnumber(n,j);
	    printf("b%d=0x%02x.%d ",j-i,b/8,b&7);
	  }
	printf("\n");
	count++;
      }
    }
  if (!count) printf("  no blocks were affected by the burst error.\n");
  return 0;
}

int main()
{
  int n;
  unsigned char in[256];
  unsigned char out[512];
  unsigned char verify[256];

  int v,e,o,b;
  int split,interleave_flag;
  
  // Make sure that golay_encode_packet() and golay_decode_packet()
  // work together properly.
  // The maximum packet size in practice is 252bytes.  Half of that
  // is golay overhead, leaving 126 bytes raw.  But 6 of those bytes
  // are header, so there is really 120 bytes of data we can stuff in
  // a packet.
  printf("Testing golay_{en,de}code_packet() reciprocality.\n");
  for(n=0;n<=120;n++)
    for(interleave_flag=0;interleave_flag<2;interleave_flag++)
      {
	prefill(in);
	setInterleaveP(interleave_flag);

	// Produce CRC and golay protected packet with netid headers and all.
	// Ends up in radio_buffer and length in radio_buffer_count
	golay_encode_packet(n,in);
	
	// Copy it into the buffer where we expect it for decoding.
	bcopy(radio_buffer,radio_interleave_buffer,radio_buffer_count);

	// Now decode it and see what we get.
	uint8_t length_out=0;
	bzero(&out[0],sizeof(out));
	int result=golay_decode_packet(&length_out,out,radio_buffer_count);
	if (!result) {
	  printf("Failed to decode encoded packet: interleave=0, n=%d\n",
		 interleave,n);
	  show("unencoded data",n,in);
	  show("encoded packet",radio_buffer_count,radio_buffer);
	  printf("packet length interpretted as = %d\n",(int)length_out);
	  show("decoded packet header (if not already overwritten)",6,out);
	  show("decoded packet",n,out);
	  verbose=1;
	  at_testmode=AT_TEST_FEC;
	  golay_encode_packet(n,in);
	  golay_decode_packet(&length_out,out,radio_buffer_count);
	  int errs=golay_decode(radio_buffer_count,radio_buffer,out);
	  printf("Packet contained %d golay errors.\n",errs);
	  show("decoded packet (including headers)",n+6,out);
	  exit(-1);
	}
      }
  printf("  -- test passed.\n");
  
  // Try interleaving and golay protecting a block of data
  // 256 bytes of golay protected data = 128 bytes of raw data.
  printf("Testing interleaving at golay_{en,de}code() level.\n");
  for(n=0;n<128;n+=3) {
    prefill(in);
    setInterleaveP(0);
    golay_encode(n,in,out);
    int icount=countones(n*2,out);
    setInterleaveP(1);
    golay_encode(n,in,out);
    int ncount=countones(n*2,out);
    if (icount!=ncount) {
      printf("Test failed: different number of set bits with/without"
	     " interleaving: %d vs %d\n",icount,ncount);
      exit(-1);
    }
    bzero(verify,256);
    int errcount=golay_decode(n*2,out,verify);
    if (bcmp(in,verify,n)||errcount) {
      printf("Decode error for packet of %d bytes (errcount=%d)\n",n,errcount);
      show("input",n,in);
      show("verify error (should be 0x00 -- 0xnn)",n,verify);

      show("interleaved encoded version",n*2,out);
      unsigned char out2[512];
      setInterleaveP(0);
      golay_encode(n,in,out2);
      show("uninterleaved encoded version",n*2,out2);
      
      int k;
      interleave_data_size=n*2;
      for(k=0;k<n*2;k++)
	out2[k]=interleave_getbyte(out,k);
      show("de-interleaved version of interleaved encoded version",n*2,out2);

      exit(-1);
    }
  }
  printf("  -- test passed.\n");
 
  // Try interleaving writing the data piece at a time using golay_encode_portion()
  // and make sure it works the same as doing the whole lot.
  printf("Testing golay_encode_portion()\n");
  for(n=0;n<128;n+=3) {
    for(interleave_flag=0;interleave_flag<2;interleave_flag++)
      for(split=n;split>=0;split-=3)
	{
	  prefill(in);
	  setInterleaveP(0);
	  golay_encode(n,in,out);
	  int icount=countones(n*2,out);
	  setInterleaveP(interleave_flag);
	  // Encode in two pieces, beginning with the last piece first
	  if (split<n) {
	    offset_start=split;
	    offset_end=n;
	    golay_encode_portion(n*2,&in[offset_start],out);
	  }
	  if (split>0) {
	    offset_start=0;
	    offset_end=split-1;
	    golay_encode_portion(n*2,&in[offset_start],out);
	  }
	  int ncount=countones(n*2,out);
	  if (icount!=ncount) {
	    printf("Test failed: different number of set bits with/without"
		   " interleaving: %d vs %d (n=%d, split=%d)\n",
		   icount,ncount,n,split);
	    show("output using golay_encode_portion()",n*2,out);
	    golay_encode(n,in,out);
	    show("output using golay_encode()",n*2,out);
	    exit(-1);
	  }
	  bzero(verify,256);
	  int errcount=golay_decode(n*2,out,verify);
	  if (bcmp(in,verify,n)||errcount) {
	    printf("Decode error for packet of %d bytes (errcount=%d)\n",n,errcount);
	    show("input",n,in);
	    show("verify error (should be 0x00 -- 0xnn)",n,verify);
	    
	    show("interleaved encoded version",n*2,out);
	    unsigned char out2[512];
	    setInterleaveP(0);
	    golay_encode(n,in,out2);
	    show("uninterleaved encoded version",n*2,out2);
	    
	    int k;
	    interleave_data_size=n*2;
	    for(k=0;k<n*2;k++)
	      out2[k]=interleave_getbyte(out,k);
	    show("de-interleaved version of interleaved encoded version",n*2,out2);
	    
	    exit(-1);
	  }
	}
  }
  printf("  -- test passed.\n");
  

  // Try interleaving and golay protecting a block of data
  // But this time, introduce errors
  printf("Testing interleaving at golay_{en,de}code() level with burst errors.\n");
  int j;
  for(n=126;n>0;n-=3) {
    // Wiping out upto 3 in every 24 encoded bits should not prevent reception
    printf("  testing n=%d bytes (%d bytes encoded), theoretically burst errors to %d bits.\n",
	   n,n*2,(n>>3)*8);
    float bestpercent=100;
    for(e=0;e<=n;e++) 
      {
	for(o=(2*n)-e-1;o>=0;o--)
	  {
	    prefill(in);
	    setInterleaveP(0);
	    golay_encode(n,in,out);
	    int icount=countones(n*2,out);
	    setInterleaveP(1);
	    golay_encode(n,in,out);
	    int ncount=countones(n*2,out);
	    if (icount!=ncount) {
	      printf("Test failed: different number of set bits with/without"
		     " interleaving: %d vs %d\n",icount,ncount);
	      exit(-1);
	    }
	    // introduce the burst error
	    for(j=o;j<o+e;j++) out[j]^=0xff;

	    // Verify that it still decodes properly
	    bzero(verify,256);
	    int errcount=golay_decode(n*2,out,verify);
	    if (bcmp(in,verify,n)) {
	      if (e>(n>>3)) {
		float percent=e*50.0/n;
		if (percent<bestpercent) bestpercent=percent;
	      } else {
		if (e>0)
		  printf("Decode error for packet of %d bytes, with burst error from 0x%02x..0x%02x bytes inclusive (%d bits = %2.1f%% of length)\n",n,o,o+e-1,
			 e*8,e*50.0/n);
		else
		  printf("Decode error for packet of %d bytes, with zero length burst error.\n",n);
		printf("  golay_decode() noticed %d errors.\n",errcount);
		show("input",n,in);
		show("verify error (should be 0x00 -- 0xnn)",n,verify);
		unsigned char out2[512];
		int k,count=0;
		for(k=0;k<n;k++) out2[k]=in[k]^verify[k];
		show("Differences",n,out2);
		
		showbitpattern(n*2,0,n*2,o*8,(o+e-1)*8+7);
		
		/* Show how the code words have been affected */
		golay_encode(n,in,out2);
		for(k=0;k<n*2;k+=6) {
		  int m;
		  int show=0;		
		  for(m=0;m<6;m++) if (interleave_getbyte(out2,k+m)!=interleave_getbyte(out,k+m)) show=1;
		  for(m=0;m<6;m++) g6[m]=interleave_getbyte(out,k+m);
		  if (golay_decode24()) show=1;
		  for(m=0;m<6;m++) g6[m]=interleave_getbyte(out2,k+m);
		  if (golay_decode24()) show=1;
		  if (show) {
		    count++;
		    printf("Golay block #%2d (without burst error) : ",k/6);
		    for(m=0;m<6;m++) printf("%02x",interleave_getbyte(out2,k+m));
		    printf("\n                   (with burst error) : ",k/6);
		    for(m=0;m<6;m++) printf("%02x",interleave_getbyte(out,k+m));
		    printf("\n");
		    for(m=0;m<6;m++) g6[m]=interleave_getbyte(out,k+m);
		    printf("  golay error count (with burst error) = %d\n",golay_decode24());
		    for(m=0;m<6;m++) g6[m]=interleave_getbyte(out2,k+m);
		    printf("  golay error count (without burst error) = %d\n",golay_decode24());
		  }
		}
		if (!count) printf("  no golay blocks were affected by the burst error.\n");
		
		exit(-1);
	      } 
	    }
	  }
      }
  printf("  ... was good to %2.1f%% (%.0f bits) wiped out\n",
	 bestpercent,bestpercent/100.0*(n*8));
  }
  printf("\n  -- test passed.\n");

  printf("Testing interleaver at low level\n");

  for(n=6;n<=510;n+=6) {
    // Perform some diagnostics on the interleaver
    bzero(out,512);
    interleave_data_size=n;
    //    printf("step=%d bits.\n",steps[n/3]);
    int i;
    for(i=0;i<n;i++) {
      interleave_setbyte(out,i,0x55);
      int r=interleave_getbyte(out,i);
      if (r!=0x55)
	{
	  printf("wrote 0x55 to byte %d of block with length = %d, but it read back as 0x%02x\n",
		 i,n,r);
	  show("encoded block",n,out);	  
	  exit(-1);
	}
      interleave_setbyte(out,i,0xaa);
      r=interleave_getbyte(out,i);
      if (r!=0xaa)
	{
	  printf("wrote 0xaa to byte %d of block with length = %d, but it read back as 0x%02x\n",
		 i,n,r);
	  show("encoded block",n,out);	  
	  exit(-1);
	}

      interleave_setbyte(out,i,0xff);
      int ones=countones(n,out);
      if (ones!=((i+1)*8)) {
	printf("Test failed for golay encoded packet size=%d\n",n);
	printf("n=%d, i=%d\n",n,i);
	show("bits clash with another byte",n,out);
	int k,l;
	for(k=0;k<(8*i);k++) 
	  for(l=0;l<8;l++) 
	    if (bitnumber(n,k)==bitnumber(n,i*8+l)) {
	      printf("  byte %d.%d (bit=%d) clashes with bit %d of this byte"
		     " @ bit %d\n",
		     k/8,k&7,k,l,bitnumber(n,i*8+l));
	      printf("them: bit*steps[n/3]%%(n*8) = %d*steps[%d]%%%d = %d\n",
		     k,n/3,n*8,k*steps[n/3]%(n*8));
	      printf("us: bit*steps[n/3]%%(n*8) = %d*steps[%d]%%%d = %d\n",
		     i*8+l,n/3,n*8,(i*8+l)*steps[n/3]%(n*8));
	    }
	for(k=0;k<8*n;k++) {
	  printf("   bit #%3d -> bit %d\n",k,bitnumber(n,k));
	  int l;
	  for(l=0;l<k;l++)
	    if (bitnumber(n,l)==bitnumber(n,k)) printf("  which is the same as for bit #%d\n",l);
	}

	exit(-1);
      }
      int j;
      for(j=0;j<=255;j++) {
	interleave_setbyte(out,i,j);
	if (interleave_getbyte(out,i)!=j) {
	  printf("Test failed for interleave_{set,get}byte(n=%d,%d) value 0x%02x\n",
		 n,i,j);
	  printf("  Expected 0x%02x, but got 0x%02x\n",j,interleave_getbyte(out,i));
	  printf("  Bits:\n");
	  int k;
	  for(k=0;k<8;k++) {
	    printf("   bit %d(%d) : %d (bit number = %d)\n",
		   k,i*8+k,
		   interleave_getbit(interleave_data_size,out,(i*8)+k),
		   bitnumber(interleave_data_size,(i*8)+k));
	  }
	  show("interleaved encoded data",n,out);
	  exit(-1);
	}
      }
    }
    printf("."); fflush(stdout);
  }
  printf("\n  -- test passed\n");

  // Make sure it can recover from upto 3 bits wrong.
  // For computational efficiency, we will only consider
  // burst errors
  printf("Testing golay coder at low level\n");
  for(v=0;v!=0x1000000;v++) {
    if (!(v&0xffff)) { printf("."); fflush(stdout); }
    for(e=0;e<4;e++)
      for(o=0;o<=(11-e);o++)
	{
	  //	  printf("v=%d, e=%d, o=%d\n",v,e,o);
	  g3[0]=v>>16;
	  g3[1]=v>>8;
	  g3[2]=v>>0;
	  golay_encode24();
	  // Flip bits to simulate errors
	  for(b=0;b<e;b++) g6[(o+b)/8]^=1<<(o+b)&7;
	  // Now try to decode
	  int errcount=golay_decode24();
	  if ((g3[0]!=((v>>16)&0xff))
	      ||(g3[1]!=((v>>8)&0xff))
	      ||(g3[2]!=((v>>0)&0xff))
	      ) {
	    printf("Bytes being protected = 0x%06x, applying %d bit errors at offset %d\n",
		   v,e,o);
	    printf("detected %d errors, when there were really %d\n",errcount,e);
	    printf("Decoded data as 0x%02x%02x%02x\n",g3[0],g3[1],g3[2]);
	    exit(-1);
	  }
	}
  }
  printf("\n  -- test passed.\n");

    
  return 0;
}
