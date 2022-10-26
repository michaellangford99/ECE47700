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
#include <stdio.h>

/*void setup_dma(void){
    //Need DMA2 Channel 3 for streams 2 and 3 (might only need the RX DMA (Stream 2))
    RCC -> AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    //Disable DMA
    DMA2_Stream2 -> CR &= ~DMA_SxCR_EN;
    //DMA2_Stream3 -> CR &= ~DMA_SxCR_EN;

    //Need to setup for both of them
    DMA2_Stream2 -> CR |= DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0;
    //DMA2_Stream3 -> CR |= DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_0;

    //16 bit word size
    DMA2_Stream2 -> CR |= DMA_SxCR_MSIZE_0;
    //DMA2_Stream3 -> CR |= DMA_SxCR_MSIZE_0;
    DMA2_Stream2 -> CR |= DMA_SxCR_PSIZE_0;
    //DMA2_Stream3 -> CR |= DMA_SxCR_PSIZE_0;
    DMA2_Stream2 -> CR |= DMA_SxCR_MINC;

    //set up peripheral address and memory storage address
    DMA2_Stream2 -> PAR = (uint32_t) (&(SPI1 -> DR));
    DMA2_Stream2 -> M0AR = (uint16_t) (data);
    //DMA2_Stream3 -> PAR = (uint16_t) (&(SPI1 -> DR));
    //DMA2_Stream3 -> M0AR = (uint16_t) (data);

    //Circular mode
    DMA2_Stream2 -> CR |= DMA_SxCR_CIRC;
    //DMA2_Stream3 -> CR |= DMA_SxCR_CIRC;
}

void enable_dma(void){
    //enable dma
    DMA2_Stream2 -> CR |= DMA_SxCR_EN;
    //DMA2_Stream3 -> CR |= DMA_SxCR_EN;
}*/

void init_spi1(void){
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;

    //using pins PA4-PA7
    GPIOA -> MODER &= ~0x0000ff00;
    GPIOA -> MODER |= 0x0000a900;

    //SPI1 is AFR5 for all functions (NSS, SCK, MOSI, MISO) except for the stupid face NSS
    GPIOA -> AFR[0] &= ~0xfff00000;
    GPIOA -> AFR[0] |= 0x55500000;

    //Very high speed GPIO PA4 with a pullup resistor for CS
    //GPIOA -> OSPEEDR |= 0x0000ff00;
    GPIOA -> PUPDR |= 0x00000200;
    GPIOA -> ODR |= (1 << 4);

    //Disable SPI1 to configure settings
    SPI1 -> CR1 &= ~SPI_CR1_SPE;

    //Setting Baud Rate (SPI clock) to the lowest possible just for testing
    SPI1 -> CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
    //Make microcontroller master
    SPI1 -> CR1 |= SPI_CR1_MSTR;
    //8bit data frame format
    SPI1 -> CR1 &= ~SPI_CR1_DFF;
    //enable slave select
    SPI1 -> CR2 |= SPI_CR2_SSOE;
    //enable software NSS
    //SPI1 -> CR1 |= SPI_CR1_SSM;
    //SPI1 -> CR1 |= SPI_CR1_SSI;
    //1 on idle
    SPI1 -> CR1 |= SPI_CR1_CPOL;
    SPI1 -> CR1 |= SPI_CR1_CPHA;
    //enable dma on rxe
    //SPI1 -> CR2 |= SPI_CR2_RXDMAEN;

    //enable interrupts
    SPI1 -> CR2 |= SPI_CR2_TXEIE;
    SPI1 -> CR2 |= SPI_CR2_RXNEIE;

    //enable SPI
    SPI1 -> CR1 |= SPI_CR1_SPE;
}

//testing spi send command function (not used)
void spi_cmd(uint16_t data){
	GPIOA -> ODR &= ~(1 << 4);
	while(!(SPI1 -> SR & SPI_SR_TXE)){;}
	SPI1 -> DR = 0xff00 & data;
	while(!(SPI1 -> SR & SPI_SR_TXE)){;}
	SPI1 -> DR = 0x00ff & data;
	while((SPI1 -> SR & SPI_SR_BSY)){;}
	GPIOA -> ODR |= 1 << 4;

}

void writeReg(uint8_t regAddress, uint8_t writeInfo){
	uint8_t write = 0x00;
	write |= regAddress;
	write &= ~(1 << 7); //tells micro to write the data in last 8 bits of DR

	GPIOA -> ODR &= ~(1 << 4);
	while(!(SPI1 -> SR & SPI_SR_TXE)){;}
	SPI1 -> DR = write;
	while(!(SPI1 -> SR & SPI_SR_TXE)){;}
	SPI1 -> DR = writeInfo;
	while((SPI1 -> SR & SPI_SR_BSY)){;}
	GPIOA -> ODR |= 1 << 4;
}

uint8_t readReg(uint8_t regAddress){
	uint8_t read;
	uint8_t write = 0x00;
	write |= regAddress;
	write |= 1 << 7; //tells LSM to write data to last 8 bits of DR

	GPIOA -> ODR &= ~(1 << 4);
	while(!(SPI1 -> SR & SPI_SR_TXE)){;}
	SPI1 -> DR = write;
	while(!(SPI1 -> SR & SPI_SR_TXE)){;}
	SPI1 -> DR = 0x00;
	while(!(SPI1 -> SR & SPI_SR_TXE)){;}
	while(!(SPI1 -> SR & SPI_SR_RXNE)){;}
	read = SPI1 -> DR;

	while((SPI1 -> SR & SPI_SR_BSY)){;}
	GPIOA -> ODR |= 1 << 4;

	return read; //8 bits of the read register
}
