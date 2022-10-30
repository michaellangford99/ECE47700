#ifndef __I2C_H__
#define __I2C_H__

void I2Cinit(void);
void I2CstartWrite(void);
void I2CstartRead(void);
void I2Cwrite (uint8_t data);
void I2Caddress(uint8_t address);
void I2Cstop(void);
void I2Cwrite2bytes (uint16_t data);
void I2CbeginTransmission(uint8_t address);
void I2CrequestFrom(uint8_t address, uint8_t *buffer, uint8_t numOfBytes);
void nano_wait(unsigned int n);
#endif /* __I2C_H__ */
