void i2c_stop(void);
void i2c_start(void);
unsigned char i2c_rx(char ack);
unsigned char i2c_tx(unsigned char d);
void i2c_clock_high(void);
void i2c_clock_low(void);
void i2c_data_high(void);
void i2c_data_low(void);

char eeprom_write_byte(unsigned short address, unsigned char value);
char eeprom_read_byte(unsigned short address, char *byte);

extern __xdata unsigned short i2c_delay_counter;
