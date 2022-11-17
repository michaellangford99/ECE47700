#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#define SYSTEM_CLOCK 	180000000//SystemCoreClock//16000000
#define APB1_PCLOCK  	(SYSTEM_CLOCK/4)
#define APB1_TIM_CLOCK 	(APB1_PCLOCK*2)
#define APB2_PCLOCK  	(SYSTEM_CLOCK/2)
#define APB2_TIM_CLOCK 	(APB2_PCLOCK*2)

#endif /* __SYSTEM_H__ */
