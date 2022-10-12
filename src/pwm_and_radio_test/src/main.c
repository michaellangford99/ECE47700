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
#include "rx_usart.h"
#include "usb_usart.h"
#include "pwm.h"

#define LED_PIN 5
#define LED_GPIO GPIOA

int main(void)
{

	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//PC13 is the LED on test board
	//PA5 is the LED on the Nucleo
	LED_GPIO->MODER |= 0x1 << (LED_PIN * 2);
	LED_GPIO->MODER &= ~(0x2 << (LED_PIN * 2));

	LED_GPIO->ODR |= 0x1 << LED_PIN;

	init_USB_USART();

	init_RX_USART();

	init_PWM();

	for(;;) {

		for (volatile int i = 99; i > 0; i--)
		{
			__asm("NOP");
		}
		LED_GPIO->ODR ^= 0x1 << LED_PIN;

		printf("%d, \t", set_PWM_duty_cycle());

		printf("%d,\t", saved_channel_data.ch0);
		printf("%d,\t", saved_channel_data.ch1);
		printf("%d,\t", saved_channel_data.ch2);
		printf("%d,\t", saved_channel_data.ch3);
		printf("%d,\t", saved_channel_data.ch4);
		printf("%d,\t", saved_channel_data.ch5);
		printf("%d,\t", saved_channel_data.ch6);
		printf("%d\n",  saved_channel_data.ch7);

		//char chr = __io_getchar();
		//printf("You entered %c.", chr);

		//printf("Enter your name: %d\r\n", 42069);
	}
}
