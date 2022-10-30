/**
  ******************************************************************************
  * @file    spi.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "string.h"
#include "spi.h"
#include "system.h"
#include <stdio.h>

void init_spi(void){
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;

    //using pins PA4-PA7
    LSM_SPI_GPIO -> MODER &= ~((0b11 << (LSM_SPI_NSS_PIN*2)) | (0b11 << (LSM_SPI_SCK_PIN*2)) | (0b11 << (LSM_SPI_MOSI_PIN*2)) | (0b11 << (LSM_SPI_MISO_PIN*2)));//~0x0000ff00;
    LSM_SPI_GPIO -> MODER |= (1 << (LSM_SPI_NSS_PIN*2)) | (2 << (LSM_SPI_SCK_PIN*2)) | (2 << (LSM_SPI_MOSI_PIN*2)) | (2 << (LSM_SPI_MISO_PIN*2));//0x0000a900;

    //LSM_SPI is AFR5 for all functions (NSS, SCK, MOSI, MISO) except for the stupid face NSS
    LSM_SPI_GPIO -> AFR[0] &= ~((0xf << (LSM_SPI_SCK_PIN*4)));
    LSM_SPI_GPIO -> AFR[0] &= ~((0xf << (LSM_SPI_MOSI_PIN*4)));
    LSM_SPI_GPIO -> AFR[0] &= ~((0xf << (LSM_SPI_MISO_PIN*4)));
    LSM_SPI_GPIO -> AFR[0] |= (5 << (LSM_SPI_SCK_PIN*4)) | (5 << (LSM_SPI_MOSI_PIN*4)) | (5 << (LSM_SPI_MISO_PIN*4));//0x55500000;

    //Very high speed GPIO PA4 with a pullup resistor for CS
    //LSM_SPI_GPIO -> OSPEEDR |= 0x0000ff00;
    LSM_SPI_GPIO -> PUPDR |= (2 << (LSM_SPI_NSS_PIN*2));//0x00000200;
    LSM_SPI_GPIO -> ODR |= (1 << LSM_SPI_NSS_PIN);

    //Disable LSM_SPI to configure settings
    LSM_SPI -> CR1 &= ~SPI_CR1_SPE;

    //Setting Baud Rate (SPI clock) to the lowest possible just for testing
    LSM_SPI -> CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
    //Make microcontroller master
    LSM_SPI -> CR1 |= SPI_CR1_MSTR;
    //8bit data frame format
    LSM_SPI -> CR1 &= ~SPI_CR1_DFF;
    //enable slave select
    LSM_SPI -> CR2 |= SPI_CR2_SSOE;
    //enable software NSS
    //LSM_SPI -> CR1 |= SPI_CR1_SSM;
    //LSM_SPI -> CR1 |= SPI_CR1_SSI;
    //1 on idle
    LSM_SPI -> CR1 |= SPI_CR1_CPOL;
    LSM_SPI -> CR1 |= SPI_CR1_CPHA;

    //enable interrupts
    LSM_SPI -> CR2 |= SPI_CR2_TXEIE;
    LSM_SPI -> CR2 |= SPI_CR2_RXNEIE;

    //enable SPI
    LSM_SPI -> CR1 |= SPI_CR1_SPE;
}

//testing spi send command function (not used)
void spi_cmd(uint16_t data){
	LSM_SPI_GPIO -> ODR &= ~(1 << LSM_SPI_NSS_PIN);
	while(!(LSM_SPI -> SR & SPI_SR_TXE)){;}
	LSM_SPI -> DR = 0xff00 & data;
	while(!(LSM_SPI -> SR & SPI_SR_TXE)){;}
	LSM_SPI -> DR = 0x00ff & data;
	while((LSM_SPI -> SR & SPI_SR_BSY)){;}
	LSM_SPI_GPIO -> ODR |= 1 << LSM_SPI_NSS_PIN;

}

void writeReg(uint8_t regAddress, uint8_t writeInfo){
	uint8_t write = 0x00;
	write |= regAddress;
	write &= ~(1 << 7); //tells micro to write the data in last 8 bits of DR

	LSM_SPI_GPIO -> ODR &= ~(1 << LSM_SPI_NSS_PIN);
	while(!(LSM_SPI -> SR & SPI_SR_TXE)){;}
	LSM_SPI -> DR = write;
	while(!(LSM_SPI -> SR & SPI_SR_TXE)){;}
	LSM_SPI -> DR = writeInfo;
	while((LSM_SPI -> SR & SPI_SR_BSY)){;}
	LSM_SPI_GPIO -> ODR |= 1 << LSM_SPI_NSS_PIN;
}

uint8_t readReg(uint8_t regAddress){
	uint8_t read;
	uint8_t write = 0x00;
	write |= regAddress;
	write |= 1 << 7; //tells LSM to write data to last 8 bits of DR

	LSM_SPI_GPIO -> ODR &= ~(1 << LSM_SPI_NSS_PIN);
	while(!(LSM_SPI -> SR & SPI_SR_TXE)){;}
	LSM_SPI -> DR = write;
	while(!(LSM_SPI -> SR & SPI_SR_TXE)){;}
	LSM_SPI -> DR = 0x00;
	while(!(LSM_SPI -> SR & SPI_SR_TXE)){;}
	while(!(LSM_SPI -> SR & SPI_SR_RXNE)){;}
	read = LSM_SPI -> DR;

	while((LSM_SPI -> SR & SPI_SR_BSY)){;}
	LSM_SPI_GPIO -> ODR |= 1 << LSM_SPI_NSS_PIN;

	return read; //8 bits of the read register
}
