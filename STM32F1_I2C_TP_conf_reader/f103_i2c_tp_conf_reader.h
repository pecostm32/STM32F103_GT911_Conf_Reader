#ifndef F103_I2C_TP_CONF_READER_H
#define F103_I2C_TP_CONF_READER_H

void printhexnibble(unsigned char nibble);

int sendi2cslaveaddres(uint8_t address, uint8_t mode);

int sendi2cbyte(uint8_t byte);

int readi2cbyte(uint8_t ack);

void readi2cconfig(uint8_t addr);

#endif /* F103_I2C_TP_CONF_READER_H */

