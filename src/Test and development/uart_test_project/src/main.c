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
#include <stdio.h>

#include "fifo.h"
#include "tty.h"

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
	DMA2_Stream2->CR &= ~DMA_SxCR_EN;	// First make sure DMA is turned off
	DMA2_Stream2->CR &= ~DMA_SxCR_CHSEL;
	DMA2_Stream2->CR |= (4 << 25);		//select channel 4

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

void USART1_IRQHandler(void)
{
	while(DMA2_Stream2->NDTR != sizeof serfifo - seroffset) {
		if (fifo_full(&input_fifo))
			fifo_remove(&input_fifo);
		insert_echo_char(serfifo[seroffset]);
		seroffset = (seroffset + 1) % sizeof serfifo;
	}
}

#define LED_PIN 13

int main(void)
{

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//PC13 is the LED
	GPIOC->MODER |= 0x1 << (LED_PIN * 2);
	GPIOC->MODER &= ~(0x2 << (LED_PIN * 2));

	GPIOC->ODR |= 0x1 << LED_PIN;

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
	USB_USART->CR1 &= ~(1 << 12);	//set bit 12, 0 for word size 8 bits
	USB_USART->CR1 &= ~USART_CR1_PCE;				//disable parity control
	USB_USART->CR2 &= ~USART_CR2_STOP;				//set stop bits to 1

	//ACTUALLY, it seems to be running at 16MHz

	//115_200 = 180MHz / (8 * 2 * USARTDIV)
	//USARTDIV = 180MHZ / (8 * 2 * 115_200)
	//USARTDIV: fraction=0d11, mantissa=0d97
	USB_USART->BRR = (7) | ((8) << 4);						//set baud rate to 115200

	USB_USART->CR1 |= USART_CR1_TE;				//enable transmitter
	USB_USART->CR1 |= USART_CR1_RE;				//enable receiver

	USB_USART->CR1 |= USART_CR1_UE;				//enable USART

	enable_USB_usart_tty_interrupt();

	setbuf(stdin,0);
	setbuf(stdout,0);
	setbuf(stderr,0);

	for(;;) {

		for (volatile int i = 999999/2; i > 0; i--)
		{
			__asm("NOP");
		}
		GPIOC->ODR ^= 0x1 << LED_PIN;

		//char chr = __io_getchar();
		//printf("You entered %c.", chr);

		//printf("Enter your name: %d\r\n", 42069);
	}
}