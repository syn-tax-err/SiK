#include "board.h"
#include "radio.h"
#include "timer.h"
#include "golay.h"
#include "crc.h"
#include "pins_user.h"
#include "sha3.h"

__xdata unsigned char eeprom_data[16];

unsigned char k;
unsigned short delay;
unsigned short i2c_delay_counter;

#if PIN_MAX>0

void i2c_delay(void)
{
  // 10 microsecond delay
  i2c_delay_counter = param_get(PARAM_I2CDELAY);
  
  // Will this be enough?
  for(delay=0;delay!=i2c_delay_counter;delay++)
    for(k=0;k!=0x0f;k++)
      continue;
    
}

void i2c_clock_high(void)
{
  pins_user_set_io(1,PIN_INPUT);
  pins_user_set_value(1,1);
}

void i2c_clock_low(void)
{
  pins_user_set_io(1,PIN_OUTPUT);
  pins_user_set_value(1,0);
}

void i2c_data_high(void)
{
  pins_user_set_io(0,PIN_INPUT);
  pins_user_set_value(0,1);
}

void i2c_data_low(void)
{
  pins_user_set_io(0,PIN_OUTPUT);
  pins_user_set_value(0,0);
}

unsigned char i2c_clock_value(void)
{
  if (pins_user_get_adc(1)) return 1; else return 0;
}

unsigned char i2c_data_value(void)
{
  if (pins_user_get_adc(0)) return 1; else return 0;
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

__xdata char x,d,timeout;

unsigned char i2c_rx(char ack)
{
  x=0; d=0; timeout=255;
  
  i2c_data_high();
  i2c_delay();

  // Receive bits
  for(x=0;x<8;x++) {
    d <<= 1;
    i2c_clock_high();
    i2c_delay();

    // Wait for any clock stretching
    while (!i2c_clock_value()) {
      timeout--; if (!timeout) return 0x54;
      i2c_delay();
    }

    if (i2c_data_value()) d|=1;

    i2c_clock_low(); i2c_delay();
  }

  // Send [n]ack
  if (ack) i2c_data_low(); else i2c_data_high();
  i2c_delay();
  
  i2c_clock_high(); i2c_delay();

  // Finish ACK/NACK
  i2c_clock_low(); i2c_delay();
  //  i2c_data_high(); i2c_delay();
  
  return d;
}

unsigned char i2c_tx(unsigned char d)
{
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
  
  return x;
}

char eeprom_read_byte(unsigned short address, char *byte)
{  
  // Setup for a write, then abort it, to set memory pointer
  i2c_start();
  if (i2c_tx(0xa0+((address>>7)&0xe))) { *byte=1; i2c_stop(); return -1; }
  if (i2c_tx(address&0xff)) { *byte=2; i2c_stop(); return -1; }

  i2c_start();
  
  if (i2c_tx(0xa1+((address>>7)&0xe))) { *byte=3; i2c_stop(); return -1; }

  *byte=i2c_rx(1);
  i2c_stop();
  return 0;
}

char eeprom_read_page(unsigned short address)
{
  i2c_start();
  if (i2c_tx(0xa0+((address>>7)&0xe))) { i2c_stop(); return -1; }
  if (i2c_tx(address&0xff)) { i2c_stop(); return -1; }

  i2c_start();
  
  if (i2c_tx(0xa1+((address>>7)&0xe))) { i2c_stop(); return -1; }

  for(unsigned char i=0;i<16;i++) eeprom_data[i]=i2c_rx(1);
  i2c_stop();
  
  return 0;
}

char eeprom_write_byte(unsigned short address, unsigned char value)
{
  i2c_start();
  if (i2c_tx(0xa0+((address>>7)&0xe))) return -1;
  if (i2c_tx(address&0xff)) return -1;
  if (i2c_tx(value)) return -1;
  i2c_stop();
  return 0;
}

char eeprom_write_page(unsigned short address)
{
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
  // This is most easily done by trying to read a byte from the EEPROM.
  // This will fail, until such time as the writing has completed.

  {
    while (eeprom_read_byte(0x0,&x)) i2c_delay();
  }

  printfl("\r\n");
  LED_ACTIVITY = LED_OFF;
  
  return 0;

 fail:
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

void eeprom_poweron(void)
{
  pins_user_set_io(2,PIN_OUTPUT);
  pins_user_set_value(2,1);
}

void eeprom_poweroff(void)
{
  pins_user_set_io(2,PIN_OUTPUT);
  pins_user_set_value(2,0);
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
  eeprom_poweron();

  puts_r("READING EEPROM\r\n");
  // putchar_r('1');
  
  // Read from $7C0-$7EF and calculate sha3 sum
  sha3_Init256();

  // (resuing i2c delay variable while out of scope)
  for(delay=0x7c0;delay<0x7F0;delay+=0x10) {
    eeprom_read_page(delay);
    sha3_Update(eeprom_data,0x10);
  }
  // Compare with sum stored at $7F0
  if (eeprom_read_page(0x7f0)) {
    // puts_r("NO EEPROM\r\n");
    // putchar_r('2');
    // No eeprom, so just use the normal saved parameters.

    eeprom_poweroff();    
    return;
  }

  // Check SHA3 sum (we use only 128 bit prefix of the hash)
  for(k=0;k<0x10;k++)
    if (eeprom_data[k]!=ctx.s[k>>3][k&7]) {
      
      // puts_r("INVALID EEPROM DATA\r\n");
      // putchar_r('3');
      
      // If we have no valid data, then we need to make sure we don't transmit
      // illegally.  
      param_set(PARAM_TXPOWER,0);
      
      eeprom_poweroff();
      return;
    }
  
  // Have valid EEPROM data.
  // Reload $7E0-$7EF, and pull out primary parameters from there
  eeprom_read_page(0x7e0);

  param_set(PARAM_TXPOWER,(eeprom_data[0]<<8)||eeprom_data[1]);
  param_set(PARAM_AIR_SPEED,(eeprom_data[2]<<8)||eeprom_data[3]);
  param_set(PARAM_FREQ,(eeprom_data[4]<<8)||eeprom_data[5]);

  // Print success message, with two-character representative country
  // code. (LBARD will read a longer country/region descriptor for
  // display).
  // puts_r("EEPROM VALID:");
  // putchar_r(eeprom_data[0xe]);
  // putchar_r(eeprom_data[0xf]);
  // puts_r("\r\n");
  //  putchar_r('4');

  eeprom_poweroff();
  return;
}
