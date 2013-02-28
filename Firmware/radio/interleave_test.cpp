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

int param_get(int param)
{
  // ECC_PARAM returns "Golay + interlacing"
  return 2;
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

int main()
{
  int n;
  unsigned char in[256];
  unsigned char out[512];

  printf("Testing interleaver\n");

  // Perform some diagnostics on the interleaver
  bzero(out,512);
  n=510;
  printf("step=%d bits.\n",steps[n/3]);
  int i;
  for(i=0;i<510;i++) {
    interleave_setbyte(n,out,i,0xff);
    show("input",n,out);
  }
  

  // Try interleaving and golay protecting a block of data
  n=255;
  prefill(in);
  show("input",n,in);
  golay_encode(n,in,out);
  show("encoded output",n,out);

}
