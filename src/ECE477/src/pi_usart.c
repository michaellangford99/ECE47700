
#include <stdint.h>
#include "stm32f4xx.h"
#include <stdio.h>
#include "system.h"
#include "pi_usart.h"
#include "fifo.h"
#include "tty.h"

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

#define PI_USART 					UART4
#define PI_USART_GPIO 				GPIOC
#define PI_USART_TX					10
#define PI_USART_RX					11
#define PI_USART_AF					8
#define PI_USART_INTERRUPT			UART4_IRQn
#define PI_USART_DMA_STREAM 		DMA1_Stream2
#define PI_USART_DMA_CHANNEL		4
#define PI_USART_INTERRUPT_HANDLE	UART4_IRQHandler

#define PI_USART_BAUDRATE			115200
#define CLOCK_RATE					SYSTEM_CLOCK
#define PI_USART_CLOCK_RATE			CLOCK_RATE/2
#define PI_USART_DIV				(PI_USART_CLOCK_RATE / (16*PI_USART_BAUDRATE))
#define PI_USART_DIV_FRACTION		(((16 * PI_USART_CLOCK_RATE / (16*PI_USART_BAUDRATE))) % 16)
#define PI_USART_DIV_MANTISSA  		PI_USART_DIV

void enable_PI_usart_rx_interrupt()
{
	PI_USART->CR1 |= USART_CR1_RXNEIE;
	PI_USART->CR3 |= USART_CR3_DMAR;

	//USART2 is interrupt 38
	NVIC->ISER[PI_USART_INTERRUPT>31] |= 1 << (PI_USART_INTERRUPT%32);

	//Set up DMA1
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	//USART1 is on DMA1 Stream 5 Channel 4
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_EN;	// First make sure DMA is turned off
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_CHSEL;
	PI_USART_DMA_STREAM->CR |= (PI_USART_DMA_CHANNEL << 25);		//select channel 4

	PI_USART_DMA_STREAM->M0AR = (uint32_t)&serfifo;
	PI_USART_DMA_STREAM->PAR = (uint32_t)&(PI_USART->DR);
	PI_USART_DMA_STREAM->NDTR = FIFOSIZE;

	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_DIR;
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_HTIE;
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_TCIE;
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_MSIZE;
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_PSIZE;
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_PINC;
	PI_USART_DMA_STREAM->CR |=  DMA_SxCR_MINC;
	PI_USART_DMA_STREAM->CR |=  DMA_SxCR_CIRC;
	PI_USART_DMA_STREAM->CR |=  DMA_SxCR_PL;

	PI_USART_DMA_STREAM->CR |= DMA_SxCR_EN;
}
/*
int __io_putchar(int c) {
	if (c == '\n') __io_putchar('\r');

	while(!(PI_USART->SR & USART_SR_TXE)) { }
	PI_USART->DR = c;
	return c;
}
*/

int interrupt_getchar()
{
	// Wait for a newline to complete the buffer.
	while(!fifo_newline(&input_fifo))
			asm volatile ("wfi"); // wait for an interrupt
	// Return a character from the line buffer.
	char ch = fifo_remove(&input_fifo);
	return ch;
}

/*
int __io_getchar(void) {
	char c = interrupt_getchar();

	return c;
}*/

void PI_USART_INTERRUPT_HANDLE(void)
{
	while(PI_USART_DMA_STREAM->NDTR != sizeof serfifo - seroffset) {
		if (fifo_full(&input_fifo))
			fifo_remove(&input_fifo);
		//insert_echo_char(serfifo[seroffset]);
		//process the data:
		
		seroffset = (seroffset + 1) % sizeof serfifo;
	}
}

void init_PI_USART(void)
{
	//Enable GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	//zero out AFR for PB6 and PB7
	PI_USART_GPIO->AFR[PI_USART_TX>7] &= ~(0xF << ((PI_USART_TX%8) *4));
	PI_USART_GPIO->AFR[PI_USART_RX>7] &= ~(0xF << ((PI_USART_RX%8) *4));

	//select AF7 for PB6 and7PB6 as USART1_TX and USART1_RX, respectively
	PI_USART_GPIO->AFR[PI_USART_TX>7] |= (PI_USART_AF << ((PI_USART_TX%8) *4));
	PI_USART_GPIO->AFR[PI_USART_RX>7] |= (PI_USART_AF << ((PI_USART_RX%8) *4));

	//zero out MODER for PB6 and PB7
	PI_USART_GPIO->MODER &= ~(0xf << (PI_USART_TX*2));
	//select alternate function mode for PB6 and PB7
	PI_USART_GPIO->MODER |= (0b1010 << (PI_USART_TX*2));

	//Enable USART4
	RCC->APB1ENR |= RCC_APB1ENR_UART4EN;

	PI_USART->CR1 &= ~USART_CR1_UE;
	PI_USART->CR1 &= ~(1 << 12);	//set bit 12, 0 for word size 8 bits
	PI_USART->CR1 &= ~USART_CR1_PCE;				//disable parity control
	PI_USART->CR2 &= ~USART_CR2_STOP;				//set stop bits to 1

	//ACTUALLY, it seems to be running at 16MHz

	//115_200 = 180MHz / (8 * 2 * USARTDIV)
	//USARTDIV = 180MHZ / (8 * 2 * 115_200)
	//USARTDIV: fraction=0d11, mantissa=0d97
	PI_USART->BRR = (PI_USART_DIV_FRACTION) | ((PI_USART_DIV_MANTISSA) << 4);						//set baud rate to 115200

	PI_USART->CR1 |= USART_CR1_TE;				//enable transmitter
	PI_USART->CR1 |= USART_CR1_RE;				//enable receiver

	PI_USART->CR1 |= USART_CR1_UE;				//enable USART

	enable_PI_usart_rx_interrupt();

	/*setbuf(stdin,0);
	setbuf(stdout,0);
	setbuf(stderr,0);*/
	//^^ would be nice to use these but with specific streams for the PI.
	//TODO: look into this
}
