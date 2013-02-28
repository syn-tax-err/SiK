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

int main()
{
  int n;
  unsigned char in[256];
  unsigned char out[512];

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
    }
  }
  printf("  -- test passed\n");

  // Try interleaving and golay protecting a block of data
  printf("Testing interleaving at golay_{en,de}code() level.\n");
  for(n=0;n<256;n+=3) {
    prefill(in);
    interleave=1;
    golay_encode(n,in,out);
    int icount=countones(n*2,out);
    interleave=0;
    golay_encode(n,in,out);
    int ncount=countones(n*2,out);
    if (icount!=ncount) {
      printf("Test failed: different number of set bits with/without"
	     " interleaving: %d vs %d\n",icount,ncount);
      exit(-1);
    }
  }

}
