#ifndef __SPI_H__
#define __SPI_H__

#define LSM_SPI 			SPI1
#define LSM_SPI_GPIO 		GPIOA
#define LSM_SPI_NSS_PIN 	4

#define SPI_GPIO            GPIOA
#define SPI_SCK_PIN 	    5
#define SPI_MOSI_PIN 	    6
#define SPI_MISO_PIN 	    7

#define MPU_SPI             SPI1
#define MPU_SPI_GPIO        GPIOC
#define MPU_SPI_NSS_PIN     4

void init_spi(void);
void spi_cmd(uint16_t data);
void writeReg(uint8_t regAddress, uint8_t writeInfo);
uint8_t readReg(uint8_t regAddress);

#endif /* __SPI_H__ */





