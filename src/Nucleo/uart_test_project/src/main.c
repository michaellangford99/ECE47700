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

void enable_tty_interrupt()
{
	USART2->CR1 |= USART_CR1_RXNEIE;
	USART2->CR3 |= USART_CR3_DMAR;

	NVIC->ISER[0] |= 1 << 29;

	RCC->AHBENR |= RCC_AHBENR_DMA2EN;
	DMA2->RMPCR |= DMA_RMPCR2_CH2_USART5_RX;
	DMA2_Channel2->CCR &= ~DMA_CCR_EN;  // First make sure DMA is turned off

	DMA2->

	DMA2_Channel2->CMAR = (uint32_t)&serfifo;
	DMA2_Channel2->CPAR = (uint32_t)&(USART5->RDR);
	DMA2_Channel2->CNDTR = FIFOSIZE;
	DMA2_Channel2->CCR &= ~DMA_CCR_DIR;
	DMA2_Channel2->CCR &= ~DMA_CCR_HTIE;
	DMA2_Channel2->CCR &= ~DMA_CCR_TCIE;
	DMA2_Channel2->CCR &= ~DMA_CCR_MSIZE;
	DMA2_Channel2->CCR &= ~DMA_CCR_PSIZE;
	DMA2_Channel2->CCR &= ~DMA_CCR_PINC;
	DMA2_Channel2->CCR |=  DMA_CCR_MINC;
	DMA2_Channel2->CCR |=  DMA_CCR_CIRC;
	DMA2_Channel2->CCR &= ~DMA_CCR_MEM2MEM;
	DMA2_Channel2->CCR |=  DMA_CCR_PL;

	DMA2_Channel2->CCR |= DMA_CCR_EN;
}

int __io_putchar(int c) {
	if (c == '\n') __io_putchar('\r');

	while(!(USART2->SR & USART_SR_TXE)) { }
	USART2->DR = c;
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
	 /*while (!(USART5->ISR & USART_ISR_RXNE)) { }
	 char c = USART5->RDR;
	 return c;*/
	char c = interrupt_getchar();

	return c;
}

void USART2_IRQHandler(void)
{
	while(DMA2_Channel2->CNDTR != sizeof serfifo - seroffset) {
		if (!fifo_full(&input_fifo))
			insert_echo_char(serfifo[seroffset]);
		seroffset = (seroffset + 1) % sizeof serfifo;
	}
}

int main(void)
{
	//USART2
	//PA2, PA3

	//Enable GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//zero out AFR for PA2 and PA3
	GPIOA->AFR[0] &= ~(0xFF << (2 *4));
	GPIOA->AFR[0] &= ~(0xFF << (3 *4));

	//select AF7 for PA2 and PA3 as USART2_TX and USART2_RX, respectively
	GPIOA->AFR[0] |= (0x07 << (2 *4));
	GPIOA->AFR[0] |= (0x07 << (3 *4));

	//zero out MODER for PA2 and PA3
	GPIOA->MODER &= ~(0xf << 4);

	//select alternate function mode for PA2 and PA3
	GPIOA->MODER |= (10 << 4);

	//Enable USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	USART2->CR1 &= ~USART_CR1_UE;
	USART2->CR1 &= ~(1 << 12);	//set bit 12, 0 for word size 8 bits
	USART2->CR1 &= ~USART_CR1_PCE;				//disable parity control
	USART2->CR2 &= ~USART_CR2_STOP;				//set stop bits to 1

	//ACTUALLY, it seems to be running at 45MHz

	//115_200 = 180MHz / (8 * 2 * USARTDIV)
	//USARTDIV = 180MHZ / (8 * 2 * 115_200)
	//USARTDIV: fraction=0d11, mantissa=0d97
	USART2->BRR = (7) | ((24) << 4);						//set baud rate to 115200

	USART2->CR1 |= USART_CR1_TE;				//enable transmitter
	USART2->CR1 |= USART_CR1_RE;				//enable receiver

	USART2->CR1 |= USART_CR1_UE;				//enable USART


	setbuf(stdin,0);
	setbuf(stdout,0);
	setbuf(stderr,0);



	for(;;) {
		//while (!(u->SR & USART_SR_RXNE)) { }
		//char c = u->DR;
		/*char idx = 0;

		while (text[idx] != 0)
		{
			while(!(u->SR & USART_SR_TXE)) { }
			u->DR = text[idx];
			idx++;
		}
		//USART2->BRR = USART2->BRR + 1;

		GPIOA->ODR ^= 0x1 << 5;
		for (volatile int i = 999999/2; i > 0; i--)
		{
			__asm("NOP");
		}*/
		printf("Enter your name: %d\r\n", 42069);
		/*while (!(u->SR & USART_SR_RXNE)) { }
		char c = u->DR;
		while(!(u->SR & USART_SR_TXE)) { }
		u->DR = c;*/



	}
}
