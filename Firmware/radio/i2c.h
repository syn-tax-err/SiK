void i2c_delay(void);
void i2c_stop(void);
void i2c_start(void);
unsigned char i2c_rx(char ack);
unsigned char i2c_tx(unsigned char d);
void i2c_clock_high(void);
void i2c_clock_low(void);
void i2c_data_high(void);
void i2c_data_low(void);

void eeprom_poweron(void);
void eeprom_poweroff(void);
char eeprom_write_byte(unsigned short address, unsigned char value);
char eeprom_read_byte(unsigned short address, char *byte);
char eeprom_read_page(unsigned short address);

extern __xdata unsigned short i2c_delay_counter;
extern __xdata unsigned char eeprom_data[16];

