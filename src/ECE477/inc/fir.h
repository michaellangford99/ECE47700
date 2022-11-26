#ifndef __FIR_H__
#define __FIR_H__

#include <stdio.h>
#include "stdint.h"
#include "system.h"
#include "math.h"

extern const float hanning_64[64];
extern const float hanning_256[256];

struct fir_filter{
	float* impulse_response;
	uint16_t length;
	float* circular_buffer;
	uint16_t first_element;
};

float compute_filter(struct fir_filter* f);
float update_filter(struct fir_filter* f, float new_value);
void shift_filter(struct fir_filter* f, float* new_values, int count);

#endif /* __FIR_H__ */
