#include "stm32f4xx.h"

#include <stdio.h>

#include "rx_usart.h"
#include "pwm.h"
#include "fifo.h"

#define PWM_GPIO	GPIOA
#define PWM_TIMR	TIM1
#define PWM_CH1_PIN	8
#define PWM_CH2_PIN 11
#define PWM_CH3_PIN 11
#define PWM_CH4_PIN 11
#define PWM_TIMR_AFR	1

// GPIOA is in this register
#define RCCAHB1ENR (RCC->AHB1ENR)
// timer 1 enable is in this register
#define RCCAPB2ENR (RCC->APB2ENR)

#define FCLK					16000000
#define PWM_REPETITION_RATE_HZ	50
#define	PWM_PSC 				15
#define PWM_ARR 				(FCLK/((PWM_PSC+1) * PWM_REPETITION_RATE_HZ) - 1)

#define LOWERBOUNDCRRX 			((PWM_ARR*5)/100)
#define UPPERBOUNDCRRX 			((PWM_ARR*10)/100)
#define RANGE					(UPPERBOUNDCRRX-LOWERBOUNDCRRX)

//consider moving towards DMA based system where timer interrupt
//causes transfer from main loop motor outputs to here

//or set the motor outputs in main loop as pointers, so they automatically affect these.
//though have to be careful as they take instantaneous effect

void set_PWM_duty_cycle() {
	PWM_TIMR->CCR1 = LOWERBOUNDCRRX + (RANGE*saved_channel_data.ch2)/1810;
	//PWM_TIMR->CCR2 = LOWERBOUNDCRRX + (RANGE*saved_channel_data.ch1)/1810;
	//PWM_TIMR->CCR3 = LOWERBOUNDCRRX + (RANGE*saved_channel_data.ch2)/1810;
	//PWM_TIMR->CCR4 = LOWERBOUNDCRRX + (RANGE*saved_channel_data.ch3)/1810;
}

void init_PWM(void)
{
	// Enable TIM1 and GPIOA
	(RCC->AHB1ENR) |= RCC_AHB1ENR_GPIOAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

	// Set the PA8 - PA12 for TIM1 CHANNELS
	PWM_GPIO->MODER &= ~(0x3 << (PWM_CH1_PIN*2));
	PWM_GPIO->MODER &= ~(0x3 << (PWM_CH2_PIN*2));
	PWM_GPIO->MODER &= ~(0x3 << (PWM_CH3_PIN*2));
	PWM_GPIO->MODER &= ~(0x3 << (PWM_CH4_PIN*2));

	PWM_GPIO->MODER |= (0x2 << (PWM_CH1_PIN*2));
	PWM_GPIO->MODER |= (0x2 << (PWM_CH2_PIN*2));
	PWM_GPIO->MODER |= (0x2 << (PWM_CH3_PIN*2));
	PWM_GPIO->MODER |= (0x2 << (PWM_CH4_PIN*2));

	// Set the alternative function register for TMR AF
	PWM_GPIO -> AFR[PWM_CH1_PIN >> 3] &= ~(0xFF << ((PWM_CH1_PIN & 0x7) * 4));
	PWM_GPIO -> AFR[PWM_CH2_PIN >> 3] &= ~(0xFF << ((PWM_CH2_PIN & 0x7) * 4));
	PWM_GPIO -> AFR[PWM_CH3_PIN >> 3] &= ~(0xFF << ((PWM_CH3_PIN & 0x7) * 4));
	PWM_GPIO -> AFR[PWM_CH4_PIN >> 3] &= ~(0xFF << ((PWM_CH4_PIN & 0x7) * 4));

	PWM_GPIO -> AFR[PWM_CH1_PIN >> 3] |= PWM_TIMR_AFR << ((PWM_CH1_PIN & 0x7) * 4);
	PWM_GPIO -> AFR[PWM_CH2_PIN >> 3] |= PWM_TIMR_AFR << ((PWM_CH2_PIN & 0x7) * 4);
	PWM_GPIO -> AFR[PWM_CH3_PIN >> 3] |= PWM_TIMR_AFR << ((PWM_CH3_PIN & 0x7) * 4);
	PWM_GPIO -> AFR[PWM_CH4_PIN >> 3] |= PWM_TIMR_AFR << ((PWM_CH4_PIN & 0x7) * 4);

	// Enable MOE bit in BDTR
	PWM_TIMR -> BDTR |= 0x00008000;

	// Set the PSC and ARR
	PWM_TIMR -> PSC = PWM_PSC;
	PWM_TIMR -> ARR = PWM_ARR;

	// Configure CCMR1 and CCMR2, to set channel 1, 2, 3, 4
	PWM_TIMR -> CCMR1 |= 0x6060;
	PWM_TIMR -> CCMR2 |= 0x6860;

	// Enable the channels
	PWM_TIMR -> CCER |= 0x1111;

	// Enable the timer
	PWM_TIMR -> CR1 |= 0x0001;

	set_PWM_duty_cycle();
}