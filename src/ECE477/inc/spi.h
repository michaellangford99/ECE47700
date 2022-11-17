#ifndef __SPI_H__
#define __SPI_H__

#include "system.h"

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

#define SPI_PCLOCK			APB2_PCLOCK

#define SPI_PCLOCK_DIV_2    (SPI_PCLOCK/2)
#define SPI_PCLOCK_DIV_4    (SPI_PCLOCK/4) 
#define SPI_PCLOCK_DIV_8    (SPI_PCLOCK/8) 
#define SPI_PCLOCK_DIV_16   (SPI_PCLOCK/16)
#define SPI_PCLOCK_DIV_32   (SPI_PCLOCK/32)
#define SPI_PCLOCK_DIV_64   (SPI_PCLOCK/64)
#define SPI_PCLOCK_DIV_128  (SPI_PCLOCK/128)
#define SPI_PCLOCK_DIV_256  (SPI_PCLOCK/256)

#define SPI_CR1_BR_PCLOCK_DIV_2		0
#define SPI_CR1_BR_PCLOCK_DIV_4	                                  SPI_CR1_BR_0
#define SPI_CR1_BR_PCLOCK_DIV_8	                   SPI_CR1_BR_1
#define SPI_CR1_BR_PCLOCK_DIV_16	               SPI_CR1_BR_1 | SPI_CR1_BR_0
#define SPI_CR1_BR_PCLOCK_DIV_32	SPI_CR1_BR_2
#define SPI_CR1_BR_PCLOCK_DIV_64	SPI_CR1_BR_2                | SPI_CR1_BR_0
#define SPI_CR1_BR_PCLOCK_DIV_128	SPI_CR1_BR_2 | SPI_CR1_BR_1
#define SPI_CR1_BR_PCLOCK_DIV_256	SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0

void init_SPI1(void);
void spi_cmd(uint16_t data);
void writeReg(uint8_t regAddress, uint8_t writeInfo);
uint8_t readReg(uint8_t regAddress);

#endif /* __SPI_H__ */





