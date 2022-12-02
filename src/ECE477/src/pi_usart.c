
#include <stdint.h>
#include "stm32f4xx.h"
#include <stdio.h>
#include "system.h"
#include "pi_usart.h"
#include "fifo.h"
#include "tty.h"
#include "lsm6ds3.h"
#include "mpu6500.h"

#define FIFOSIZE 16
char pi_serfifo[FIFOSIZE];
int pi_seroffset = 0;

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
#define CLOCK_RATE					APB1_PCLOCK
#define PI_USART_CLOCK_RATE			CLOCK_RATE
#define PI_USART_DIV				(PI_USART_CLOCK_RATE / (16*PI_USART_BAUDRATE))
#define PI_USART_DIV_FRACTION		(((16 * PI_USART_CLOCK_RATE / (16*PI_USART_BAUDRATE))) % 16)
#define PI_USART_DIV_MANTISSA  		PI_USART_DIV

void enable_PI_usart_rx_interrupt()
{
	PI_USART->CR1 |= USART_CR1_RXNEIE;
	PI_USART->CR3 |= USART_CR3_DMAR;

	//USART2 is interrupt 38
	NVIC->ISER[PI_USART_INTERRUPT/32] |= 1 << (PI_USART_INTERRUPT%32);

	//Set up DMA1
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

	//USART1 is on DMA1 Stream 5 Channel 4
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_EN;	// First make sure DMA is turned off
	PI_USART_DMA_STREAM->CR &= ~DMA_SxCR_CHSEL;
	PI_USART_DMA_STREAM->CR |= (PI_USART_DMA_CHANNEL << 25);		//select channel 4

	PI_USART_DMA_STREAM->M0AR = (uint32_t)&pi_serfifo;
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

void pi_putchar(uint8_t c)
{
	if (c == '\n') pi_putchar('\r');

	while(!(PI_USART->SR & USART_SR_TXE)) { }
	PI_USART->DR = c;
}

void pi_puts(uint8_t* s)
{
	while (*s != '\0')
	{
		pi_putchar(*s);
		s++;
	}
}

pi_packet_t saved_pi_packet;

#define BUFFER_SIZE 60
uint8_t pi_buffer[BUFFER_SIZE*2];
uint8_t* pi_buffer_start = pi_buffer;
#define PI_FRAME_START_CODE 'A'
#define PI_FRAME_END_CODE 'B'

uint8_t process_pi_packet_buffer(uint8_t new)
{
	//first, add the byte and shift the
	*pi_buffer_start = new;
	pi_buffer_start++;
	if ((uint32_t)(pi_buffer_start-pi_buffer) > BUFFER_SIZE-1)
		pi_buffer_start = pi_buffer;

	if ((*pi_buffer_start) == PI_FRAME_START_CODE)
	{
		//go through and attempt to parse
		//place the data struct at this location and see if it passes
		//first task is to verify the length is within bounds, and
		//that the message type is what we're looking for
		//Then verify it has a valid CRC
		//move everything from the beginning of memory to

		pi_packet_t* pi_packet = (pi_packet_t*)pi_buffer_start;

		//copy everything from bottom to the pi_buffer_start point into the upper half of the array to make it contiguous
		memcpy(pi_buffer + BUFFER_SIZE, pi_buffer, pi_buffer_start-pi_buffer);

		//instead, validate the ending sequence
		//or validate CRC
		if (pi_packet->ending == PI_FRAME_END_CODE)
		{
		//uint8_t calculated_crc = calc_crc(&(pi_packet->header.type), sizeof(crsf_channels_t)+1/*don't include the CRC in the CRC calc haha*/);
		//uint8_t rx_crc = pi_packet->crc;

		//if (rx_crc == calculated_crc)
		//{
			//valid packet
			memcpy((void*)&saved_pi_packet, (void*)pi_packet, sizeof(pi_packet_t));
			return 1;
		//}
		}
	}
	//if any check here is false, the data gets shifted until a valid packet is detected
	return 0;
}


void PI_USART_INTERRUPT_HANDLE(void)
{
	while(PI_USART_DMA_STREAM->NDTR != sizeof pi_serfifo - pi_seroffset) {
		
		uint8_t parse_status = process_pi_packet_buffer(pi_serfifo[pi_seroffset]);

		if (parse_status)
		{
			//crsf_channels_t* channel_data = &saved_channel_data;

			//instead, save the LiDAR and steering commands

			//printf("Saved a sequence!");

			/*printf("%d,\t", channel_data->ch0);
			printf("%d,\t", channel_data->ch1);
			printf("%d,\t", channel_data->ch2);
			printf("%d,\t", channel_data->ch3);
			printf("%d,\t", channel_data->ch4);
			printf("%d,\t", channel_data->ch5);
			printf("%d,\t", channel_data->ch6);
			printf("%d\n", channel_data->ch7);*/
		}
		
		//putchar('0'+ pi_serfifo[pi_seroffset]);

		//pi_putchar(pi_serfifo[pi_seroffset]);
		pi_seroffset = (pi_seroffset + 1) % sizeof pi_serfifo;
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

	//log startup details:
	printf("PI_USART:\n");
	printf("\tPI_USART_BAUDRATE:      %d\n", PI_USART_BAUDRATE);
	printf("\tCLOCK_RATE:             %d\n", CLOCK_RATE);
	printf("\tPI_USART_CLOCK_RATE:    %d\n", PI_USART_CLOCK_RATE);
	printf("\tPI_USART_DIV:           %d\n", PI_USART_DIV);
	printf("\tPI_USART_DIV_FRACTION:  %d\n", PI_USART_DIV_FRACTION);
	printf("\tPI_USART_DIV_MANTISSA:  %d\n", PI_USART_DIV_MANTISSA);
	printf("\tPI_FRAME_START_CODE:    %d\n", PI_FRAME_START_CODE);
	printf("\tPI_FRAME_END_CODE:      %d\n", PI_FRAME_END_CODE);
}

char output_text[100];

void test_PI_USART()
{
	sprintf(output_text, "Testing PI USART.\t%d\n", 420);
	pi_puts(output_text);
}

int pi_update_count = 0;

void update_PI_USART()
{
	pi_update_count++;
	if (pi_update_count >= 300)
	{
		pi_update_count = 0;

		sprintf(output_text, "IMU\t%.2f,\t%.2f,\t%.2f\n", lsm6dsx_data.gyro_angle_z, lsm6dsx_data.compl_pitch, lsm6dsx_data.compl_pitch);
		pi_puts(output_text);
	}
}
