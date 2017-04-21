#include <stdbool.h>
#include "board.h"
#include "radio.h"
#include "timer.h"
#include "golay.h"
#include "crc.h"
#include "pins_user.h"
#include "sha3.h"
#include "i2c.h"

bool eeprom_param_request=false;
__xdata unsigned char eeprom_data[16];

unsigned char k;
unsigned short delay;
unsigned short i2c_delay_counter;

#if PIN_MAX>0

#if 0
void i2c_delay(void)
{
  // 10 microsecond delay
  // i2c_delay_counter = param_get(PARAM_I2CDELAY);
  
  // Will this be enough?
  //  for(delay=0;delay!=i2c_delay_counter;delay++)
  //  for(k=0;k!=0x0f;k++)
  //  continue;

  // actually, it turns out that just calling this function and reading
  // the I2CDELAY parameter is sufficient delay.
    
}
#else
#define i2c_delay()
#endif

// Prior to Rev4 Mesh Extender PCB, clock was IO1, now IO4
void i2c_clock_high(void)
{
  pins_user_set_io(4,PIN_INPUT);
  pins_user_set_value(4,1);
}

void i2c_clock_low(void)
{
  pins_user_set_io(4,PIN_OUTPUT);
  pins_user_set_value(4,0);
}

// Prior to Rev4 Mesh Extender PCB, data was IO0, now IO3
void i2c_data_high(void)
{
  pins_user_set_io(3,PIN_INPUT);
  pins_user_set_value(3,1);
}

void i2c_data_low(void)
{
  pins_user_set_io(3,PIN_OUTPUT);
  pins_user_set_value(3,0);
}

unsigned char i2c_clock_value(void)
{
  if (pins_user_get_adc(4)) return 1; else return 0;
}

unsigned char i2c_data_value(void)
{
  if (pins_user_get_adc(3)) return 1; else return 0;
}


void i2c_stop(void)
{
  i2c_data_low();  i2c_delay();
  i2c_clock_high(); i2c_delay();
  i2c_data_high();  i2c_delay();

}

void i2c_start(void)
{
  i2c_data_high();  i2c_delay();
  i2c_clock_high(); i2c_delay();
  
  i2c_data_low();  i2c_delay();
}

char x,d,timeout,read_error;

unsigned char i2c_rx(char ack)
{
  x=0; d=0; timeout=255;
  read_error=0;
  
  // Receive bits
// #if defined BOARD_rfd900p
  #if 0
  // Optimised reading routine for RFD900p

  // data = input, float high
  SFRPAGE = CONFIG_PAGE; P1DRV   |= 0x8;
  SFRPAGE = LEGACY_PAGE; P1MDOUT &= ~0x8;
  P1|=0x8;
  
  for(x=0;x<8;x++) {
    d <<= 1;
    P1 |= 2; // clock high
    if (P1&8) d|=1;
    P1 &= 2; // clock low
  }

  // Send [n]ack
  P1MDOUT|=8;
  if (ack) P1&=~8;
  else P1|=8;

  // Finish ACK/NACK
  P1 |= 2; // clock high
  for(x=0;x<1;x++) continue;
  P1 &= ~2; // clock low
  
#else
  i2c_data_high();

  for(x=0;x<8;x++) {
    d <<= 1;
    i2c_clock_high();

    // Wait for any clock stretching
    while (!i2c_clock_value()) {
      timeout--; if (!timeout) {
	read_error=0x54;
	return 0x54;
      }
    }

    if (i2c_data_value()) d|=1;

    i2c_clock_low();
  }
  
  // Send [n]ack
  if (ack) i2c_data_low(); else i2c_data_high();
  
  i2c_clock_high(); 

  // Finish ACK/NACK
  i2c_clock_low(); 

#endif
  
  return d;
}

unsigned char i2c_tx(unsigned char d)
{

#if 0
  // #if defined board_rfd900p
  P1 &= ~2; // clock low

  for(x=8;x;x--) {
    if (d&0x80) P1 |= 8; else P1 &= ~8;
    d<<=1;
    P1 |= 2; // clock high
    for(readerror=0;readerror<2;readerror++) continue;
    P1 &= ~2; // clock low
  }

  // data = input, float high
  SFRPAGE = CONFIG_PAGE; P1DRV   |= 8;
  SFRPAGE = LEGACY_PAGE; P1MDOUT &= ~8;
  P1|=8;

  P1 |= 8; // clock high

  if (P1 & 8) x=1; else x=0;

  P1 &= ~2; // clock low
  
      
#else
  i2c_clock_low(); i2c_delay();
  
  for(x=8;x;x--) {
    if (d&0x80) i2c_data_high(); else i2c_data_low();
    d<<=1;
    i2c_delay();
    i2c_clock_high(); i2c_delay();
    i2c_clock_low(); i2c_delay();
  }

  // Re-float data so that we can read ACK
  i2c_data_high(); i2c_delay();
  i2c_clock_high(); i2c_delay();

  // Read ACK
  x=i2c_data_value();

  // Close ACK clock pulse
  i2c_clock_low(); i2c_delay();
#endif
  
  return x;
}

char eeprom_read_byte(unsigned short address, char *byte)
{  
  // Setup for a write, then abort it, to set memory pointer
  i2c_start();
  if (i2c_tx(0xa0+((address>>7)&0xe))) { i2c_stop(); return 1; }
  if (i2c_tx(address&0xff)) { i2c_stop(); return 2; }

  i2c_start();
  
  if (i2c_tx(0xa1+((address>>7)&0xe))) { i2c_stop(); return 3; }

  *byte=i2c_rx(1);
  i2c_stop();
  return 0;
}

// Currently takes ~75ms
char eeprom_read_page(unsigned short address)
{
  i2c_start();
  if (i2c_tx(0xa0|((address>>7)&0xe))) { i2c_stop(); return 4; }
  if (i2c_tx(address&0xff)) { i2c_stop(); return 5; }
  
  i2c_start();

  return eeprom_read_next_page(address);
}
  
char eeprom_read_next_page(unsigned short address)
{
  if (i2c_tx(0xa1+((address>>7)&0xe))) { i2c_stop(); return 6; }
  
  for(unsigned char i=0;i<15;i++) {
    eeprom_data[i]=i2c_rx(1);
    if (read_error) {
      i2c_stop();
      return read_error;
    }
  }

  // Terminate the sequential read
  eeprom_data[15]=i2c_rx(0);
  if (read_error) {
    i2c_stop();
    return read_error;
  }
  
  i2c_stop();
  
  return 0;
}

char eeprom_write_byte(unsigned short address, unsigned char value)
{
  uint8_t waiting=1;

  i2c_start();
  if (i2c_tx(0xa0+((address>>7)&0xe))) {  i2c_stop(); return -1; }
  if (i2c_tx(address&0xff))  {  i2c_stop(); return -1; }
  if (i2c_tx(value))  {  i2c_stop(); return -1; }
  i2c_stop();

  // Now wait until the EEPROM has finished writing.
  // This is most easily done by trying to WRITE a byte from the EEPROM.
  // This will fail, until such time as the writing has completed.

  while(waiting)
  {
    i2c_start();
    if (!i2c_tx(0xa0)) // dummy write instruction
      {
	waiting=0;
	i2c_stop();
      }    
  }
  
  return 0;
}

char eeprom_write_page(unsigned short address)
{
  uint8_t waiting=1;
  i2c_start();

  // Due to slowness, show pretty lights while writing
  LED_RADIO = LED_ON;
  LED_ACTIVITY = LED_ON;
  
  if (i2c_tx(0xa0+((address>>7)&0xe))) goto fail;
  if (i2c_tx(address&0xff)) goto fail;
  for(char i=0;i<16;i++) {
    if (i2c_tx(eeprom_data[i])) goto fail;
    printfl(" %x",eeprom_data[i]);

    if (i&1) {
      LED_RADIO = LED_ON;
      LED_ACTIVITY = LED_OFF;
    } else {
      LED_RADIO = LED_OFF;
      LED_ACTIVITY = LED_ON;
    }
    
  }
  i2c_stop();
  LED_RADIO = LED_ON;
  LED_ACTIVITY = LED_ON;
  
  // Now wait until the EEPROM has finished writing.
  // This is most easily done by trying to WRITE a byte from the EEPROM.
  // This will fail, until such time as the writing has completed.

  waiting=1;
  while(waiting)
  {
    i2c_start();
    if (!i2c_tx(0xa0)) // dummy write instruction
      {
	waiting=0;
	i2c_stop();
      }    
  }

  printfl("\r\n");
  LED_ACTIVITY = LED_OFF;
  
  return 0;

 fail:
  i2c_stop();
  LED_RADIO = LED_ON;
  LED_ACTIVITY = LED_OFF;
  return -1;
}

void eeprom_writeprotect(void)
{
  pins_user_set_io(5,PIN_INPUT);
  pins_user_set_value(5,1);
}

void eeprom_writeenable(void)
{
  pins_user_set_io(5,PIN_OUTPUT);
  pins_user_set_value(5,0);
}

// Prior to Rev4 Mesh Extender PCB, poweron was IO2, now IO0
void eeprom_poweron(void)
{
  pins_user_set_io(0,PIN_OUTPUT);
  pins_user_set_value(0,1);
}

void eeprom_poweroff(void)
{
  pins_user_set_io(0,PIN_OUTPUT);
  pins_user_set_value(0,0);
}

#else
// No GPIOs, so ignore

void eeprom_poweron(void)
{
  return;
}

void eeprom_poweroff(void)
{
  return;
}

char eeprom_write_byte(unsigned short address, unsigned char value)
{
  address=value;  // suppress compiler warnings
  return -1;
}

char eeprom_read_byte(unsigned short address, char *byte)
{  
  *byte="NOEPROM."[address&7];
  return -1;
}

char eeprom_read_page(unsigned short address)
{
  address++; // suppress compiler warning
  return -1;
}

char eeprom_write_page(unsigned short address)
{
  address++; // suppress compiler warning
  return -1;
}


#endif

void eeprom_load_parameters(void)
{
  uint16_t a;
  
//  printf("READING EEPROM");
  
  eeprom_poweron();
  
  // Read from $7C0-$7EF and calculate sha3 sum
  sha3_Init256();

  // (resuing i2c delay variable while out of scope)
  for(a=0x7c0;a<0x7F0;a+=0x10) {
    eeprom_read_page(a);
    sha3_Update(eeprom_data,0x10);
 //   printf(".");
  }
 // printf("\r\n");

  // Compare with sum stored at $7F0
  if (eeprom_read_page(0x7f0)) {
  //  printf("NO EEPROM\r\n");
    // No eeprom, so just use the normal saved parameters.

    eeprom_poweroff();    
    return;
  }

  sha3_Finalize();
  
  // Check SHA3 sum (we use only 128 bit prefix of the hash)
  for(k=0;k<0x10;k++) {
    if (eeprom_data[k]!=ctx.s[k>>3][k&7]) {
      
      printf("INVALID EEPROM DATA\r\n");
      
      // If we have no valid data, then we need to make sure we don't transmit
      // illegally.  
      param_set(PARAM_TXPOWER,0);
      
      eeprom_poweroff();
      return;
    }
  }
  
  // Have valid EEPROM data.
  // Reload $7E0-$7EF, and pull out primary parameters from there
  eeprom_read_page(0x7e0);

  if (eeprom_data[0xf]==0x1) {
    uint32_t v;
    param_set(PARAM_TXPOWER,eeprom_data[1]);
    param_set(PARAM_DUTY_CYCLE,eeprom_data[0]);
    param_set(PARAM_AIR_SPEED,(eeprom_data[7]<<8)||eeprom_data[6]);
    v =((uint32_t)eeprom_data[5]<<24);
    v|=(((uint32_t)eeprom_data[4])<<16);
    v|=(((uint32_t)eeprom_data[3])<<8);
    v|=(((uint32_t)eeprom_data[2])<<0);
    param_set(PARAM_FREQ,v);
    
    // Print success message, with two-character representative country
    // code. (LBARD will read a longer country/region descriptor for
    // display).
    printf("EEPROM VALID: %c%c, %lu Hz\r\n",
	   eeprom_data[0xD],eeprom_data[0xE],
	   (unsigned long)v,
	   eeprom_data[5],eeprom_data[4],eeprom_data[3],eeprom_data[2]);
  } else {
    printf("EEPROM DATA FORMAT %x unknown\r\n",eeprom_data[0xf]);
  }

  eeprom_poweroff();
  return;
}
