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
			

#define LED_PIN 13

int main(void)
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//PA5 is the LED

	GPIOC->MODER |= 0x1 << (LED_PIN * 2);
	GPIOC->MODER &= ~(0x2 << (LED_PIN * 2));

	GPIOC->ODR |= 0x1 << LED_PIN;

	while (1)
	{
		GPIOC->ODR ^= 0x1 << LED_PIN;
		for (volatile int i = 999999/2; i > 0; i--)
		{
			__asm("NOP");
		}
	}
}
