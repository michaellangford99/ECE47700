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

// GPIOA is in this register
#define RCCAHB1ENR (RCC->AHB1ENR)
// timer 1 enable is in this register
#define RCCAPB2ENR (RCC->APB2ENR)

// Correct value  PSC Register value to achieve cycletime = 20ms (ROUGH)
#define PSCGLORY (200-1)

// Correct value  ARR Register value to achieve cycletime = 20ms (ROUGH)
#define ARRGLORY (10200-1)

#define LOWERBOUNDCRRX (24 * (100 - 58))

#define UPPERBOUNDCRRX (24 * (100 - 79))

void init_tim1(void) {
    // Enable TIM1 and GPIOA
	(RCC->AHB1ENR) |= RCC_AHB1ENR_GPIOAEN;
    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

    // Set the PA8 - PA12 for TIM1 CHANNELS
    GPIOA -> MODER  &= ~0x00ff0000;
    GPIOA -> MODER  |= 0x00aa0000;

    // Set the alternative function register for AF1

    GPIOA -> AFR[1] |= 0x00001111;
    // Enable MOE bit in BDTR
    TIM1 -> BDTR |= 0x00008000;

    // Set the PSC and ARR
    TIM1 -> PSC = PSCGLORY;
    TIM1 -> ARR = ARRGLORY;

    // Configure CCMR1 and CCMR2, to set channel 1, 2, 3, 4
    TIM1 -> CCMR1 |= 0x00006060;
    TIM1 -> CCMR2 |= 0x00006860;

    // Enable the channels
    TIM1 -> CCER |= 0x00001111;

    // Enable the timer
    TIM1 -> CR1 |= 0x00000001;

}

void setrgb(int rgb) {
    int r = ((rgb >> 20) & ~0xfffff0) * 10 + ((rgb >> 16) & ~0xfffff0);
    int g = ((rgb >> 12) & ~0xfffff0) * 10 + ((rgb >> 8) & ~0xfffff0);
    int b = ((rgb >> 4) & ~0xfffff0) * 10 + ((rgb) & ~0xfffff0);
    TIM1 -> CCR1 = UPPERBOUNDCRRX;
    TIM1 -> CCR2 = LOWERBOUNDCRRX;
    TIM1 -> CCR3 = LOWERBOUNDCRRX;
}

int main(void)
{

    // LAB 7 part
    init_tim1();

    int key = 0xffffff00;
    for(;;) {
            setrgb(key);
            if(key > 0x99999900) {
            	key = 0x50505000;
            }
    }
}
