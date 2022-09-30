#include "stm32f4xx.h"

#include <stdio.h>

#include "RX_usart.h"
#include "fifo.h"
#include "tty.h"

#define FIFOSIZE 26
char rx_serfifo[FIFOSIZE];
int rx_seroffset = 0;

#define ELRS_PACKET_START_BYTE	0xC8
#define ELRS_PACKET_MAX_SIZE 	40
char active_packet[ELRS_PACKET_MAX_SIZE];
int  active_packet_offset;
int  is_packet_active;

#define RX_USART 					USART1
#define RX_USART_GPIO 				GPIOA
#define RX_USART_TX					9
#define RX_USART_RX					10
#define RX_USART_INTERRUPT			37
#define RX_USART_DMA_STREAM 		DMA2_Stream2
#define RX_USART_DMA_CHANNEL		4
#define RX_USART_INTERRUPT_HANDLE	USART1_IRQHandler

void enable_rx_usart_interrupt()
{
	RX_USART->CR1 |= USART_CR1_RXNEIE;
	RX_USART->CR3 |= USART_CR3_DMAR;

	//USART2 is interrupt 38
	NVIC->ISER[1] |= 1 << (RX_USART_INTERRUPT-32);

	//Set up DMA1
	//RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	//USART1 is on DMA1 Stream 5 Channel 4
	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_EN;	// First make sure DMA is turned off
	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_CHSEL;
	RX_USART_DMA_STREAM->CR |= (RX_USART_DMA_CHANNEL << 25);		//select channel 4

	RX_USART_DMA_STREAM->M0AR = (uint32_t)&rx_serfifo;
	RX_USART_DMA_STREAM->PAR = (uint32_t)&(RX_USART->DR);
	RX_USART_DMA_STREAM->NDTR = FIFOSIZE;

	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_DIR;
	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_HTIE;
	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_TCIE;
	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_MSIZE;
	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_PSIZE;
	RX_USART_DMA_STREAM->CR &= ~DMA_SxCR_PINC;
	RX_USART_DMA_STREAM->CR |=  DMA_SxCR_MINC;
	RX_USART_DMA_STREAM->CR |=  DMA_SxCR_CIRC;
	RX_USART_DMA_STREAM->CR |=  DMA_SxCR_PL;

	RX_USART_DMA_STREAM->CR |= DMA_SxCR_EN;
}

/*int __io_putchar(int c) {
	if (c == '\n') __io_putchar('\r');

	while(!(RX_USART->SR & USART_SR_TXE)) { }
	RX_USART->DR = c;
	return c;
}
*/

/*
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
}*/

void check_packet(void)
{

}

void shift_packet(void)
{

}

//Start out wioth some psuedo code
//using a circular buffer, new bytes get written
//to the buffer one after another, until length is met.
//then old bytes are overwritten from the beginning.
//a pointer / index exists indicating the starting byte
//everytime a new byte arrives, the pointer is incremented, and the corresponding byte overwritten.
#define BUFFER_SIZE ELRS_PACKET_MAX_SIZE*2
uint8_t rx_buffer[BUFFER_SIZE];
uint8_t* rx_buffer_start = rx_buffer;

void process_packet_buffer(uint8_t new)
{
	//first, add the byte and shift the
	*rx_buffer_start = new;
	rx_buffer_start++;
	if (rx_buffer_start > BUFFER_SIZE-1)
	{

	}

}


void RX_USART_INTERRUPT_HANDLE(void)
{
	while(RX_USART_DMA_STREAM->NDTR != sizeof rx_serfifo - rx_seroffset) {

		if (is_packet_active == 0)
		{
			if (rx_serfifo[rx_seroffset] == 0xC8)
			{
				is_packet_active = 1;
				active_packet_offset = 0;
				active_packet[active_packet_offset] = rx_serfifo[rx_seroffset];
				active_packet_offset++;
			}
		}
		else
		{
			active_packet[active_packet_offset] = rx_serfifo[rx_seroffset];
			active_packet_offset++;

			/*if (active_packet[1] < 23 && (active_packet_offset > 2))
			{
				is_packet_active = 1;
				active_packet_offset = 0;
			}
			else */if ((active_packet_offset > 2) && active_packet_offset >= (active_packet[1] + 2))
			{

				crsf_channels_t* channel_data;

				channel_data = (crsf_channels_t*)(active_packet + 3);

				if (active_packet_offset > 24)
				{

					//saved_channel_data.ch0 = channel_data->ch0;
					//saved_channel_data.ch1 = channel_data->ch1;
					//saved_channel_data.ch2 = channel_data->ch2;

					memcpy(&saved_channel_data, channel_data, sizeof(crsf_channels_t));

					printf("%d,\t", channel_data->ch0);
					printf("%d,\t", channel_data->ch1);
					printf("%d,\t", channel_data->ch2);
					printf("%d,\t", channel_data->ch3);
					printf("%d,\t", channel_data->ch4);
					printf("%d,\t", channel_data->ch5);
					printf("%d,\t", channel_data->ch6);
					printf("%d\n", channel_data->ch7);
				}

				//we done
				is_packet_active = 0;
				active_packet_offset = 0;
			}
		}


		/*if (rx_serfifo[rx_seroffset] == 0xC8)
		{
			packet_count++;
			printf("packet - count=%d, NDTR=%d\n", packet_count, RX_USART_DMA_STREAM->NDTR);
		}*/

		rx_seroffset = (rx_seroffset + 1) % sizeof rx_serfifo;
	}
}

void init_RX_USART(void)
{
	//USART1
	//PB6, PB7

	//USART2
	//PA2 (TX), PA3 (RX)

	//Enable GPIOA
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	RX_USART_GPIO->AFR[1] &= ~(0xF << (RX_USART_TX *4 - 32));
	RX_USART_GPIO->AFR[1] &= ~(0xF << (RX_USART_RX *4 - 32));

	RX_USART_GPIO->AFR[1] |= (0x07 << (RX_USART_TX *4 - 32));
	RX_USART_GPIO->AFR[1] |= (0x07 << (RX_USART_RX *4 - 32));

	//zero out MODER for PB6 and PB7
	RX_USART_GPIO->MODER &= ~(0xf << (RX_USART_TX*2));
	//select alternate function mode for PB6 and PB7
	RX_USART_GPIO->MODER |= (10 << (RX_USART_TX*2));


	//Enable USART1
	//RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	//Enable USART2
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	RX_USART->CR1 &= ~USART_CR1_UE;
	RX_USART->CR1 &= ~(1 << 12);	//set bit 12, 0 for word size 8 bits
	RX_USART->CR1 &= ~USART_CR1_PCE;				//disable parity control
	RX_USART->CR2 &= ~USART_CR2_STOP;				//set stop bits to 1

	//ACTUALLY, it seems to be running at 16MHz

	//115_200 = 180MHz / (8 * 2 * USARTDIV)
	//USARTDIV = 180MHZ / (8 * 2 * 115_200)
	//USARTDIV: fraction=0d11, mantissa=0d97
	RX_USART->BRR = (6) | ((2) << 4);						//set baud rate to 115200

	RX_USART->CR1 |= USART_CR1_TE;				//enable transmitter
	RX_USART->CR1 |= USART_CR1_RE;				//enable receiver

	RX_USART->CR1 |= USART_CR1_UE;				//enable USART

	enable_rx_usart_interrupt();
}
