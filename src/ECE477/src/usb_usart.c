#include "stm32f4xx.h"

#include <stdio.h>
#include "system.h"
#include "usb_usart.h"
#include "fifo.h"
#include "tty.h"

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

#define USB_USART 				USART1
#define USB_USART_GPIO 			GPIOB
#define USB_USART_TX			6
#define USB_USART_RX			7
#define USB_USART_INTERRUPT		37
#define USB_USART_DMA_STREAM 	DMA2_Stream5
#define USB_USART_DMA_CHANNEL	4
#define USB_USART_INTERRUPT_HANDLE	USART1_IRQHandler

#define USB_USART_BAUDRATE		115200
#define CLOCK_RATE				SYSTEM_CLOCK
#define USB_USART_CLOCK_RATE	CLOCK_RATE/2
#define USB_USART_DIV			(USB_USART_CLOCK_RATE / (16*USB_USART_BAUDRATE))
#define USB_USART_DIV_FRACTION	(((16 * USB_USART_CLOCK_RATE / (16*USB_USART_BAUDRATE))) % 16)
#define USB_USART_DIV_MANTISSA  USB_USART_DIV

void enable_USB_usart_tty_interrupt()
{
	USB_USART->CR1 |= USART_CR1_RXNEIE;
	USB_USART->CR3 |= USART_CR3_DMAR;

	//USART2 is interrupt 38
	NVIC->ISER[USB_USART_INTERRUPT>31] |= 1 << (USB_USART_INTERRUPT%32);

	//Set up DMA1
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	//USART1 is on DMA1 Stream 5 Channel 4
	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_EN;	// First make sure DMA is turned off
	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_CHSEL;
	USB_USART_DMA_STREAM->CR |= (USB_USART_DMA_CHANNEL << 25);		//select channel 4

	USB_USART_DMA_STREAM->M0AR = (uint32_t)&serfifo;
	USB_USART_DMA_STREAM->PAR = (uint32_t)&(USB_USART->DR);
	USB_USART_DMA_STREAM->NDTR = FIFOSIZE;

	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_DIR;
	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_HTIE;
	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_TCIE;
	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_MSIZE;
	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_PSIZE;
	USB_USART_DMA_STREAM->CR &= ~DMA_SxCR_PINC;
	USB_USART_DMA_STREAM->CR |=  DMA_SxCR_MINC;
	USB_USART_DMA_STREAM->CR |=  DMA_SxCR_CIRC;
	USB_USART_DMA_STREAM->CR |=  DMA_SxCR_PL;

	USB_USART_DMA_STREAM->CR |= DMA_SxCR_EN;
}

int __io_putchar(int c) {
	if (c == '\n') __io_putchar('\r');

	while(!(USB_USART->SR & USART_SR_TXE)) { }
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

void USB_USART_INTERRUPT_HANDLE(void)
{
	while(USB_USART_DMA_STREAM->NDTR != sizeof serfifo - seroffset) {
		if (fifo_full(&input_fifo))
			fifo_remove(&input_fifo);
		insert_echo_char(serfifo[seroffset]);
		seroffset = (seroffset + 1) % sizeof serfifo;
	}
}

void init_USB_USART(void)
{
	//Enable GPIO
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//zero out AFR for PB6 and PB7
	USB_USART_GPIO->AFR[0] &= ~(0xF << (USB_USART_TX *4));
	USB_USART_GPIO->AFR[0] &= ~(0xF << (USB_USART_RX *4));

	//select AF7 for PB6 and7PB6 as USART1_TX and USART1_RX, respectively
	USB_USART_GPIO->AFR[0] |= (0x07 << (USB_USART_TX *4));
	USB_USART_GPIO->AFR[0] |= (0x07 << (USB_USART_RX *4));

	//zero out MODER for PB6 and PB7
	USB_USART_GPIO->MODER &= ~(0xf << (USB_USART_TX*2));
	//select alternate function mode for PB6 and PB7
	USB_USART_GPIO->MODER |= (10 << (USB_USART_TX*2));

	//Enable USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USB_USART->CR1 &= ~USART_CR1_UE;
	USB_USART->CR1 &= ~(1 << 12);	//set bit 12, 0 for word size 8 bits
	USB_USART->CR1 &= ~USART_CR1_PCE;				//disable parity control
	USB_USART->CR2 &= ~USART_CR2_STOP;				//set stop bits to 1

	//ACTUALLY, it seems to be running at 16MHz

	//115_200 = 180MHz / (8 * 2 * USARTDIV)
	//USARTDIV = 180MHZ / (8 * 2 * 115_200)
	//USARTDIV: fraction=0d11, mantissa=0d97
	USB_USART->BRR = (USB_USART_DIV_FRACTION) | ((USB_USART_DIV_MANTISSA) << 4);						//set baud rate to 115200

	USB_USART->CR1 |= USART_CR1_TE;				//enable transmitter
	USB_USART->CR1 |= USART_CR1_RE;				//enable receiver

	USB_USART->CR1 |= USART_CR1_UE;				//enable USART

	enable_USB_usart_tty_interrupt();

	setbuf(stdin,0);
	setbuf(stdout,0);
	setbuf(stderr,0);
}
