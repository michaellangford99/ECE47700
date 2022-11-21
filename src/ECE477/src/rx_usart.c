#include "stm32f4xx.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "system.h"
#include "crc8.h"
#include "rx_usart.h"
#include "fifo.h"
#include "tty.h"

#define FIFOSIZE 26
char rx_serfifo[FIFOSIZE];
int rx_seroffset = 0;

#define RX_USART 					USART6
#define RX_USART_GPIO 				GPIOC
#define RX_USART_TX					6
#define RX_USART_RX					7
#define RX_USART_AF					8
#define RX_USART_INTERRUPT			71
#define RX_USART_DMA_STREAM 		DMA2_Stream2
#define RX_USART_DMA_CHANNEL		5
#define RX_USART_INTERRUPT_HANDLE	USART6_IRQHandler

#define RX_USART_BAUDRATE		CRSF_RX_BAUDRATE
#define CLOCK_RATE				APB2_PCLOCK
#define RX_USART_CLOCK_RATE		CLOCK_RATE
#define RX_USART_DIV			(RX_USART_CLOCK_RATE / (16*RX_USART_BAUDRATE))
#define RX_USART_DIV_FRACTION	(((16 * RX_USART_CLOCK_RATE / (16*RX_USART_BAUDRATE))) % 16)
#define RX_USART_DIV_MANTISSA	RX_USART_DIV

crsf_channels_t saved_channel_data;

#define ELRS_PACKET_MAX_SIZE 	40
#define BUFFER_SIZE ELRS_PACKET_MAX_SIZE
uint8_t rx_buffer[BUFFER_SIZE*2];
uint8_t* rx_buffer_start = rx_buffer;

void enable_rx_usart_interrupt()
{
	RX_USART->CR1 |= USART_CR1_RXNEIE;
	RX_USART->CR3 |= USART_CR3_DMAR;

	NVIC->ISER[RX_USART_INTERRUPT/32] |= 1 << (RX_USART_INTERRUPT%32);

	//Set up DMA2
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

//Start out with some psuedocode
//using a circular buffer, new bytes get written
//to the buffer one after another, until length is met.
//then old bytes are overwritten from the beginning.
//a pointer / index exists indicating the starting byte
//everytime a new byte arrives, the pointer is incremented, and the corresponding byte overwritten.

uint8_t process_packet_buffer(uint8_t new)
{
	//first, add the byte and shift the
	*rx_buffer_start = new;
	rx_buffer_start++;
	if ((uint32_t)(rx_buffer_start-rx_buffer) > BUFFER_SIZE-1)
		rx_buffer_start = rx_buffer;

	if ((crsf_addr_e)(*rx_buffer_start) == CRSF_ADDRESS_FLIGHT_CONTROLLER)
	{
		//go through and attempt to parse
		//place the data struct at this location and see if it passes
		//first task is to verify the length is within bounds, and
		//that the message type is what we're looking for
		//Then verify it has a valid CRC
		//move everything from the beginning of memory to

		crsf_packet_t* crsf_packet = (crsf_packet_t*)rx_buffer_start;

		if (crsf_packet->header.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
		{
			if (crsf_packet->header.frame_size == sizeof(crsf_channels_t)+2/*type, crc, and payload*/)
			{
				//validate CRC

				//copy everything from bottom to the rx_buffer_start point into the upper half of the array to make it contiguous
				memcpy(rx_buffer + BUFFER_SIZE, rx_buffer, rx_buffer_start-rx_buffer);

				uint8_t calculated_crc = calc_crc(&(crsf_packet->header.type), sizeof(crsf_channels_t)+1/*don't include the CRC in the CRC calc haha*/);
				uint8_t rx_crc = crsf_packet->crc;

				if (rx_crc == calculated_crc)
				{
					//valid packet
					//TODO save the last 30 or so channel packets for filtering if needed
					memcpy((void*)&saved_channel_data, (void*)&(crsf_packet->channels), sizeof(crsf_channels_t));
					return 1;
				}
			}
		}
	}
	//if any check here is false, the data gets shifted until a valid packet is detected
	return 0;
}


void RX_USART_INTERRUPT_HANDLE(void)
{
	while(RX_USART_DMA_STREAM->NDTR != sizeof rx_serfifo - rx_seroffset) {

		uint8_t parse_status = process_packet_buffer(rx_serfifo[rx_seroffset]);

		if (parse_status)
		{
			crsf_channels_t* channel_data = &saved_channel_data;

			/*printf("%d,\t", channel_data->ch0);
			printf("%d,\t", channel_data->ch1);
			printf("%d,\t", channel_data->ch2);
			printf("%d,\t", channel_data->ch3);
			printf("%d,\t", channel_data->ch4);
			printf("%d,\t", channel_data->ch5);
			printf("%d,\t", channel_data->ch6);
			printf("%d\n", channel_data->ch7);*/
		}

		rx_seroffset = (rx_seroffset + 1) % sizeof rx_serfifo;
	}
}

crsf_channels_t* RX_USART_get_channels()
{
	return &saved_channel_data;
}

uint16_t RX_USART_convert_channel_to_motor_range(uint16_t ch)
{
	return (uint16_t)((((uint32_t)65535)*(uint32_t)(ch - CRSF_CHANNEL_VALUE_MIN))/(uint32_t)(CRSF_CHANNEL_VALUE_MAX-CRSF_CHANNEL_VALUE_MIN));
}

void init_RX_USART(void)
{
	init_crc8(0xd5);

	//Enable GPIO
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//zero out AFR for PB6 and PB7
	RX_USART_GPIO->AFR[RX_USART_TX>7] &= ~(0xF << ((RX_USART_TX%8) *4));
	RX_USART_GPIO->AFR[RX_USART_RX>7] &= ~(0xF << ((RX_USART_RX%8) *4));

	//select AF7 for PB6 and7PB6 as USART1_TX and USART1_RX, respectively
	RX_USART_GPIO->AFR[RX_USART_TX>7] |= (RX_USART_AF << ((RX_USART_TX%8) *4));
	RX_USART_GPIO->AFR[RX_USART_RX>7] |= (RX_USART_AF << ((RX_USART_RX%8) *4));

	//zero out MODER for PB6 and PB7
	RX_USART_GPIO->MODER &= ~(0xf << (RX_USART_TX*2));
	//select alternate function mode for PB6 and PB7
	RX_USART_GPIO->MODER |= (10 << (RX_USART_TX*2));

	//Enable USART6
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

	RX_USART->CR1 &= ~USART_CR1_UE;
	RX_USART->CR1 &= ~(1 << 12);	//set bit 12, 0 for word size 8 bits
	RX_USART->CR1 &= ~USART_CR1_PCE;				//disable parity control
	RX_USART->CR2 &= ~USART_CR2_STOP;				//set stop bits to 1

	//ACTUALLY, it seems to be running at 16MHz

	//115_200 = 180MHz / (8 * 2 * USARTDIV)
	//USARTDIV = 180MHZ / (8 * 2 * 115_200)
	//USARTDIV: fraction=0d11, mantissa=0d97
	RX_USART->BRR = (RX_USART_DIV_FRACTION) | ((RX_USART_DIV_MANTISSA) << 4);						//set baud rate to 115200

	RX_USART->CR1 |= USART_CR1_TE;				//enable transmitter
	RX_USART->CR1 |= USART_CR1_RE;				//enable receiver

	RX_USART->CR1 |= USART_CR1_UE;				//enable USART

	enable_rx_usart_interrupt();

	//log startup details:
	printf("RX_USART:\n");
	printf("\tRX_USART_BAUDRATE:      %d\n", RX_USART_BAUDRATE);
	printf("\tCLOCK_RATE:             %d\n", CLOCK_RATE);
	printf("\tRX_USART_CLOCK_RATE:    %d\n", RX_USART_CLOCK_RATE);
	printf("\tRX_USART_DIV:           %d\n", RX_USART_DIV);
	printf("\tRX_USART_DIV_FRACTION:  %d\n", RX_USART_DIV_FRACTION);
	printf("\tRX_USART_DIV_MANTISSA:  %d\n", RX_USART_DIV_MANTISSA);
	printf("\tCRSF_RX_BAUDRATE:       %d\n", CRSF_RX_BAUDRATE);
	printf("\tCRSF_NUM_CHANNELS:      %d\n", CRSF_NUM_CHANNELS);
	printf("\tCRSF_CHANNEL_VALUE_MIN: %d\n", CRSF_CHANNEL_VALUE_MIN);
	printf("\tCRSF_CHANNEL_VALUE_MID: %d\n", CRSF_CHANNEL_VALUE_MID);
	printf("\tCRSF_CHANNEL_VALUE_MAX: %d\n", CRSF_CHANNEL_VALUE_MAX);
	printf("\tCRSF_MAX_PACKET_LEN:    %d\n", CRSF_MAX_PACKET_LEN);
}
