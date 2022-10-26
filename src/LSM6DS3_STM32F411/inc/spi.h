#ifndef __SPI_H__
#define __SPI_H__

void init_spi1(void);
void spi_cmd(uint16_t data);
void writeReg(uint8_t regAddress, uint8_t writeInfo);
uint8_t readReg(uint8_t regAddress);

#endif /* __SPI_H__ */





