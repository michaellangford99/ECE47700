/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f4xx.h"

//LSM6DSO definitions for register addresses and pins in the register
#define FUNC_CFG_ACCESS 0x01;
#define WHO_AM_I 0x0f; //read only
#define FIFO_CTRL1 0x07;
#define FIFO_CTRL2 0x08;
#define FIFO_CTRL3 0x09;
#define FIFO_CTRL4 0x0a;
#define CTRL1_XL 0x10;
#define CTRL2_G 0x11;
#define CTRL3_G 0x12;
#define CTRL4_G 0x13;
#define CTRL5_G 0x14;
#define CTRL6_G 0x15;
#define CTRL7_G 0x16;
#define CTRL8_XL 0x17;
#define CTRL9_XL 0x18;
#define CTRL10_C 0x19;
#define OUT_TEMP_L 0x20; //read only
#define OUT_TEMP_H 0x21; //read only
#define OUTX_L_G 0x22; //read only
#define OUTX_H_G 0x23; //read only
#define OUTY_L_G 0x24; //read only
#define OUTY_H_G 0x25; //read only
#define OUTZ_L_G 0x26; //read only
#define OUTZ_H_G 0x27; //read only
#define OUTX_L_A 0x28; //read only
#define OUTX_H_A 0x29; //read only
#define OUTY_L_A 0x2a; //read only
#define OUTY_H_A 0x2b; //read only
#define OUTZ_L_A 0x2c; //read only
#define OUTZ_H_A 0x2d; //read only

uint16_t data[6] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

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

    GPIOA -> ODR |= (1 << 4);

    //Disable SPI1 to configure settings
    SPI1 -> CR1 &= ~SPI_CR1_SPE;

    //Setting Baud Rate (SPI clock) to the lowest possible just for testing
    SPI1 -> CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2;
    //Make microcontroller master
    SPI1 -> CR1 |= SPI_CR1_MSTR;
    //16bit data frame format
    SPI1 -> CR1 |= SPI_CR1_DFF;
    //enable slave select
    SPI1 -> CR2 |= SPI_CR2_SSOE;
    //enable software NSS
    //SPI1 -> CR1 |= SPI_CR1_SSM;
    //SPI1 -> CR1 |= SPI_CR1_SSI;
    //1 on idle
    SPI1 -> CR1 |= SPI_CR1_CPOL;
    //SPI1 -> CR1 |= SPI_CR1_CPHA;
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
	while((SPI1 -> SR & SPI_SR_TXE) == 0){;}
	GPIOA -> ODR &= ~(1 << 4);
	SPI1 -> DR = data;
	while((SPI1 -> SR & SPI_SR_TXE) == 0){;}
	GPIOA -> ODR |= (1 << 4);

}

void writeReg(uint8_t regAddress, uint8_t writeInfo){
	uint16_t write = 0x0000;
	write |= (regAddress << 8);
	write |= writeInfo;
	write &= ~(1 << 15); //tells micro to write the data in last 8 bits of DR

	while((SPI1 -> SR & SPI_SR_TXE) == 0){;}
	SPI1 -> DR = write;
}

uint8_t readReg(uint8_t regAddress){
	uint8_t read;
	uint16_t write = 0x0000;
	write |= (regAddress << 8);
	write |= (1 << 15); //tells LSM to write data to last 8 bits of DR

	while((SPI1 -> SR & SPI_SR_TXE) == 0){;}
	SPI1 -> DR  = write;

	while((SPI1 -> SR & SPI_SR_RXNE)){;}
	read = 0x00ff & (SPI1 -> DR);

	return read; //8 bits of the read register
}

void initLSM(){
	//register addresses (see define statements)
	uint8_t addr1 = CTRL1_XL;
	uint8_t addr2 = CTRL2_G;
	uint8_t addr3 = CTRL3_G;
	uint8_t addr4 = CTRL4_G;
	uint8_t addr5 = CTRL5_G;
	uint8_t addr6 = CTRL6_G;
	uint8_t addr7 = CTRL7_G;
	uint8_t addr8 = CTRL8_XL;
	uint8_t addr9 = CTRL9_XL;
	uint8_t addr10 = CTRL10_C;

	//inital LSM6DSO parameters (see LSM6DSO reference manual (https://www.st.com/en/mems-and-sensors/lsm6dsr.html))
	uint8_t ctrl1 = 0b1010000;
	uint8_t ctrl2 = 0b10100000;
	uint8_t ctrl3 = 0b00000000;
	uint8_t ctrl4 = 0b00000000;
	uint8_t ctrl5 = 0b00000000;
	uint8_t ctrl6 = 0b00000000;
	uint8_t ctrl7 = 0b00000000;
	uint8_t ctrl8 = 0b00000000;
	uint8_t ctrl9 = 0b11100010;
	uint8_t ctrl10 = 0b00000000;

	//ctrl register writing
	writeReg(addr1, ctrl1);
	writeReg(addr2, ctrl2);
	writeReg(addr3, ctrl3);
	writeReg(addr4, ctrl4);
	writeReg(addr5, ctrl5);
	writeReg(addr6, ctrl6);
	writeReg(addr7, ctrl7);
	writeReg(addr8, ctrl8);
	writeReg(addr9, ctrl9);
	writeReg(addr10, ctrl10);
}

int main(void)
{
	//dma is disabled as it is not really used (everything is being done manually anyways)
    //setup_dma();
    //enable_dma();
    init_spi1();
    //initLSM();
    while(1){
    	spi_cmd(0x0000);
    	spi_cmd(0xffff);
    }
    /*
    uint8_t xgyroL = OUTX_L_G;
    uint8_t xgyroH = OUTX_H_G;
    uint8_t ygyroL = OUTY_L_G;
    uint8_t ygyroH = OUTY_H_G;
    uint8_t zgyroL = OUTZ_L_G;
    uint8_t zgyroH = OUTZ_H_G;

    uint8_t xaccelL = OUTX_L_A;
    uint8_t xaccelH = OUTX_H_A;
    uint8_t yaccelL = OUTY_L_A;
    uint8_t yaccelH = OUTY_H_A;
    uint8_t zaccelL = OUTZ_L_A;
    uint8_t zaccelH = OUTZ_H_A;

    while(1){
    	data[0] = (readReg(xgyroH) << 8);
    	data[0] |= readReg(xgyroL);
    	data[1] = (readReg(ygyroH) << 8);
    	data[1] |= readReg(ygyroL);
    	data[2] = (readReg(zgyroH) << 8);
    	data[2] |= readReg(zgyroL);
    	data[3] = (readReg(xgyroH) << 8);
    	data[3] |= readReg(xgyroL);
    	data[4] = (readReg(ygyroH) << 8);
    	data[4] |= readReg(ygyroL);
    	data[5] = (readReg(zgyroH) << 8);
    	data[5] |= readReg(zgyroL);
    }*/
    	return 0;
}
