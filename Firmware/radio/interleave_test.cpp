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

int interleave=1;
int param_get(int param)
{
  // ECC_PARAM returns "Golay + interlacing"
  if (interleave)
    return 2;
  else return 1;
}

#include "interleave.c"
#include "golay.c"

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
	    printf("b7=0x%02x.%d ",b/8,b&7);
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

  printf("Testing interleaver at low level\n");

  for(n=6;n<=510;n+=6) {
    // Perform some diagnostics on the interleaver
    bzero(out,512);
    interleave_data_size=n;
    //    printf("step=%d bits.\n",steps[n/3]);
    int i;
    for(i=0;i<n;i++) {
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
  printf("  -- test passed\n");

  // Try interleaving and golay protecting a block of data
  // 256 bytes of golay protected data = 128 bytes of raw data.
  printf("Testing interleaving at golay_{en,de}code() level.\n");
  for(n=0;n<128;n+=3) {
    prefill(in);
    interleave=0;
    golay_encode(n,in,out);
    int icount=countones(n*2,out);
    interleave=1;
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
      interleave=0;
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
 
  // Try interleaving and golay protecting a block of data
  // But this time, introduce errors
  printf("Testing interleaving at golay_{en,de}code() level with burst errors.\n");
  int e,o,j;
  for(n=126;n>0;n-=3) {
    // Wiping out upto 1/4 of the bytes should not prevent reception
    for(e=0;e<(n*2/4)-1;e++) 
      {
	for(o=(2*n)-e-1;o>=0;o--)
	  {
	    prefill(in);
	    interleave=0;
	    golay_encode(n,in,out);
	    int icount=countones(n*2,out);
	    interleave=1;
	    golay_encode(n,in,out);
	    int ncount=countones(n*2,out);
	    if (icount!=ncount) {
	      printf("Test failed: different number of set bits with/without"
		     " interleaving: %d vs %d\n",icount,ncount);
	      exit(-1);
	    }
	    // introduce the burst error
	    for(j=o;j<o+e;j++) out[j]=0;

	    // Verify that it still decodes properly
	    bzero(verify,256);
	    int errcount=golay_decode(n*2,out,verify);
	    if (bcmp(in,verify,n)) {
	      if (e>0)
		printf("Decode error for packet of %d bytes, with burst error from 0x%02x..0x%02x bytes inclusive (%2f%% of length)\n",n,o,o+e-1,
		       e*50.0/n);
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
	    } else printf(".");
	  }
      }
  }
  printf("  -- test passed.\n");
    
  return 0;
}
