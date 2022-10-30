
#include "stm32f4xx.h"
#include "string.h"
#include "systick.h"
#include "system.h"

#define AHB_CLOCK 		SYSTEM_CLOCK		//48 MHz
#define AHB_CLOCK_DIV_8 (AHB_CLOCK/8)	//6 MHz
#define SYSTICK_INT_FREQ 1000			//1KHz desired interrupt frequency
#define SYSTICK_LOAD	((AHB_CLOCK_DIV_8/SYSTICK_INT_FREQ)-1)

#define LED_PIN 5
#define LED_GPIO GPIOA

void init_SYSTICK()
{
	SysTick->LOAD = SYSTICK_LOAD;
	SysTick->VAL = 0;

	SysTick->CTRL |= (1<<0);
	SysTick->CTRL |= (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk/* | SysTick_CTRL_CLKSOURCE_Msk*/);
}

uint32_t ticks = 0;
void SysTick_Handler(void)
{
	ticks++;
}

uint32_t millis()
{
	return ticks;
}

uint32_t ahb_clock_ticks()
{
	return SysTick->VAL;
}

float ftime()
{
	return ((float)ticks / (float)SYSTICK_INT_FREQ) + (float)(SYSTICK_LOAD-SysTick->VAL) / (float)AHB_CLOCK_DIV_8;
}

void nano_wait(uint32_t count)
{
	for (volatile uint32_t i = count; i > 0; i--)
	{
		__asm("NOP");
	}
}

void wait(float seconds)
{
	float t0 = ftime();
	while (ftime()-t0 < seconds);
}

