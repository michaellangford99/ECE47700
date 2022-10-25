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

#define G_GAIN 0.00875; //gyroscope gain to convert to degrees per second
#define A_GAIN 0.00061; //accelerometer gain to convert to g's??
//USART Defines
/*#define USB_USART               USART2
/#define USB_USART_GPIO          GPIOA
#define USB_USART_TX            2
#define USB_USART_RX            3
#define USB_USART_INTERRUPT     38
#define USB_USART_DMA_STREAM    DMA1_Stream5
#define USB_USART_DMA_CHANNEL   4
#define USB_USART_INTERRUPT_HANDLE  USART2_IRQHandler*/

uint16_t data[6] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

#include "fifo.h"
#include "tty.h"
#include <stdio.h>

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

#define USB_USART USART1

void enable_USB_usart_tty_interrupt()
{
    USB_USART->CR1 |= USART_CR1_RXNEIE;
    USB_USART->CR3 |= USART_CR3_DMAR;

    //USART1 is interrupt 37
    NVIC->ISER[1] |= 1 << (37-32);

    //Set up DMA2
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    //USART1 is on DMA2 Stream 2 Channel 4
    DMA2_Stream2->CR &= ~DMA_SxCR_EN;   // First make sure DMA is turned off
    DMA2_Stream2->CR &= ~DMA_SxCR_CHSEL;
    DMA2_Stream2->CR |= (4 << 25);      //select channel 4

    DMA2_Stream2->M0AR = (uint32_t)&serfifo;
    DMA2_Stream2->PAR = (uint32_t)&(USB_USART->DR);
    DMA2_Stream2->NDTR = FIFOSIZE;

    DMA2_Stream2->CR &= ~DMA_SxCR_DIR;
    DMA2_Stream2->CR &= ~DMA_SxCR_HTIE;
    DMA2_Stream2->CR &= ~DMA_SxCR_TCIE;
    DMA2_Stream2->CR &= ~DMA_SxCR_MSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_PSIZE;
    DMA2_Stream2->CR &= ~DMA_SxCR_PINC;
    DMA2_Stream2->CR |=  DMA_SxCR_MINC;
    DMA2_Stream2->CR |=  DMA_SxCR_CIRC;
    DMA2_Stream2->CR |=  DMA_SxCR_PL;

    DMA2_Stream2->CR |= DMA_SxCR_EN;
}

int __io_putchar(int c) {
    if (c == '\n') __io_putchar('\r');

    while(!(USB_USART->SR & USART_SR_TXE)) {}
    USB_USART->DR = c;
    return c;
}


int interrupt_getchar()
{
    // Wait for a newline to complete the buffer.
    while(!fifo_newline(&input_fifo))
            asm volatile ("wfi"); // wait for an interrupt
    // Return a character from the line buffer.
    char ch = fifo_remove(&input_fifo);
    return ch;
}

int __io_getchar(void) {
    char c = interrupt_getchar();

    return c;
}

void USART1_IRQHandler(void)
{
    while(DMA2_Stream2->NDTR != sizeof serfifo - seroffset) {
        if (fifo_full(&input_fifo))
            fifo_remove(&input_fifo);
        insert_echo_char(serfifo[seroffset]);
        seroffset = (seroffset + 1) % sizeof serfifo;
    }
}

void init_UART(void){
        //USART1
        //PB6, PB7

        //Enable GPIOB
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

        //zero out AFR for PB6 and PB7
        GPIOB->AFR[0] &= ~(0xF << (6 *4));
        GPIOB->AFR[0] &= ~(0xF << (7 *4));

        //select AF7 for PB6 and7PB6 as USART1_TX and USART1_RX, respectively
        GPIOB->AFR[0] |= (0x07 << (6 *4));
        GPIOB->AFR[0] |= (0x07 << (7 *4));

        //zero out MODER for PB6 and PB7
        GPIOB->MODER &= ~(0xf << 12);
        //select alternate function mode for PB6 and PB7
        GPIOB->MODER |= (10 << 12);


        //Enable USART1
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        USB_USART->CR1 &= ~USART_CR1_UE;
        USB_USART->CR1 &= ~(1 << 12);   //set bit 12, 0 for word size 8 bits
        USB_USART->CR1 &= ~USART_CR1_PCE;               //disable parity control
        USB_USART->CR2 &= ~USART_CR2_STOP;              //set stop bits to 1

        //ACTUALLY, it seems to be running at 16MHz

        //115_200 = 180MHz / (8 * 2 * USARTDIV)
        //USARTDIV = 180MHZ / (8 * 2 * 115_200)
        //USARTDIV: fraction=0d11, mantissa=0d97
        USB_USART->BRR = (7) | ((8) << 4);                      //set baud rate to 115200

        USB_USART->CR1 |= USART_CR1_TE;             //enable transmitter
        USB_USART->CR1 |= USART_CR1_RE;             //enable receiver

        USB_USART->CR1 |= USART_CR1_UE;             //enable USART

        enable_USB_usart_tty_interrupt();

        setbuf(stdin,0);
        setbuf(stdout,0);
        setbuf(stderr,0);
}

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

void initLSM(){
	//register addresses (see define statements)
	//uint8_t addr0 = FUNC_CFG_ACCESS;
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
	//uint8_t ctrl0 = 0b10000000;
	uint8_t ctrl1 = 0b00010000;
	uint8_t ctrl2 = 0b00010000;
	uint8_t ctrl3 = 0b00000000;
	uint8_t ctrl4 = 0b00000100;
	uint8_t ctrl5 = 0b00000000;
	uint8_t ctrl6 = 0b00000000;
	uint8_t ctrl7 = 0b00000000;
	uint8_t ctrl8 = 0b00000000;
	uint8_t ctrl9 = 0b11100010;
	uint8_t ctrl10 = 0b00000000;

	//ctrl register writing
	//writeReg(addr0, ctrl0);
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
    init_spi1();
    initLSM();
    init_UART();
    /*while(1){
    	uint8_t whoami = OUTX_L_G;
    	uint8_t read;
    	read = readReg(whoami);
        printf("Hello");
    	for(int i = 0; i < 10000; i++){
    	    asm("NOP");
    	}
    }*/
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

    /*while(1){
        uint8_t inte = readReg(xgyroL);
        printf("%d\n", inte);
        //printf("%d\n", data[1]);
        //printf("%d\n", data[2]);
        //printf("%d\n", data[3]);
        //printf("%d\n", data[4]);
        //printf("%d\n", data[5]);
    }*/

    while(1){
        data[0] = (readReg(xgyroH) | readReg(xgyroL) << 8) * G_GAIN;
        data[1] = (readReg(ygyroH) | readReg(ygyroL) << 8) * G_GAIN;
        data[2] = (readReg(zgyroH) | readReg(zgyroL) << 8) * G_GAIN;
        data[3] = (readReg(xaccelH) | readReg(xaccelL) << 8) * A_GAIN;
        data[4] = (readReg(yaccelH) | readReg(yaccelL) << 8) * A_GAIN;
        data[5] = (readReg(zaccelH) | readReg(zaccelL) << 8) * A_GAIN;

        /*data[0] = (readReg(xgyroH) << 8);
    	data[0] |= readReg(xgyroL);
    	data[1] = (readReg(ygyroH) << 8);
    	data[1] |= readReg(ygyroL);
    	data[2] = (readReg(zgyroH) << 8);
    	data[2] |= readReg(zgyroL);
    	data[3] = (readReg(xaccelH) << 8);
    	data[3] |= readReg(xaccelL);
    	data[4] = (readReg(yaccelH) << 8);
    	data[4] |= readReg(yaccelL);
    	data[5] = (readReg(zaccelH) << 8);
    	data[5] |= readReg(zaccelL);*/

    	//printf("%d\n", data[0]);
    	//printf("%d\n", data[1]);
    	//printf("%d\n", data[2]);
    	//printf("%d\n", data[3]);
    	//printf("%d\n", data[4]);
    	//printf("%d\n", data[5]);

    	printf("Gyro Values: X=%d Y=%d Z=%d\n", data[0], data[1], data[2]);
    	printf("Accel Values: X=%d Y=%d Z=%d\n", data[3], data[4], data[5]);

    	for(int i = 0; i < 100000; i++){
    	    __asm("NOP");
    	}

    	for(int j = 0; j < sizeof(data); j++){
    	    data[j] = 0;
    	}

    }
    return 0;
}
