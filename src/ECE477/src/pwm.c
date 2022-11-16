#include "stm32f4xx.h"

#include <stdio.h>

#include "system.h"
#include "rx_usart.h"
#include "pwm.h"
#include "fifo.h"

#define PWM_GPIO		GPIOA
#define PWM_TIMR		TIM1
#define PWM_CH1_PIN		8
#define PWM_CH2_PIN 	9
#define PWM_CH3_PIN 	10
#define PWM_CH4_PIN 	11
#define PWM_TIMR_AFR	1

// GPIOA is in this register
#define RCCAHB1ENR (RCC->AHB1ENR)
// timer 1 enable is in this register
#define RCCAPB2ENR (RCC->APB2ENR)

#define FCLK					SYSTEM_CLOCK
#define PWM_REPETITION_RATE_HZ	50*8 //Oneshot125 pulse frequency
#define	PWM_PSC 				1
#define PWM_ARR 				(FCLK/((PWM_PSC+1) * PWM_REPETITION_RATE_HZ) - 1)

#define LOWERBOUNDCRRX 			((PWM_ARR*5)/100)
#define UPPERBOUNDCRRX 			((PWM_ARR*10)/100)
#define RANGE					(UPPERBOUNDCRRX-LOWERBOUNDCRRX)

#define FIR_LENGTH 400
uint32_t last[FIR_LENGTH];

//consider moving towards DMA based system where timer interrupt
//causes transfer from main loop motor outputs to here

//or set the motor outputs in main loop as pointers, so they automatically affect these.
//though have to be careful as they take instantaneous effect


//takes as input an array of 4 uint16_t values and scales it into it's valid range
void set_PWM_duty_cycle(pwm_output_t pwm_output) {
	PWM_TIMR->CCR1 = LOWERBOUNDCRRX + (RANGE*(uint32_t)pwm_output.duty_cycle_ch0)/(100000);
	PWM_TIMR->CCR2 = LOWERBOUNDCRRX + (RANGE*(uint32_t)pwm_output.duty_cycle_ch1)/(100000);
	PWM_TIMR->CCR3 = LOWERBOUNDCRRX + (RANGE*(uint32_t)pwm_output.duty_cycle_ch2)/(100000);
	PWM_TIMR->CCR4 = LOWERBOUNDCRRX + (RANGE*(uint32_t)pwm_output.duty_cycle_ch3)/(100000);
}

void init_PWM(void)
{
	for (int i = 0; i < FIR_LENGTH; i++)
	{
		last[i] = 0;
	}

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

	printf("RANGE: %d\n", RANGE);

	//set_PWM_duty_cycle();
}
