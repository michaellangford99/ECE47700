#ifndef __PWM_H__
#define __PWM_H__

#include <stdio.h>
#include "stdint.h"
#include "system.h"
#include "math.h"

typedef struct {
	uint16_t duty_cycle_ch0;
	uint16_t duty_cycle_ch1;
	uint16_t duty_cycle_ch2;
	uint16_t duty_cycle_ch3;
} pwm_output_t;

void init_PWM(void);

void set_PWM_duty_cycle();

#endif /* __PWM_H__ */
