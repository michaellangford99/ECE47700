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
#include "usb_usart.h"
#include "spi.h"
#include "lsm6ds3.h"
#include "systick.h"

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
	//init_RX_USART();
	//init_PWM();
	init_SYSTICK();
	init_SPI1();
	init_LSM6DS3();

	test_LSM6DS3();

	for (;;)
	{

	}
}
