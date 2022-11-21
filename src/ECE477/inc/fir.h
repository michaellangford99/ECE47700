#ifndef __FIR_H__
#define __FIR_H__

extern const float hanning_64[64];
extern const float hanning_256[256];

struct fir_filter{
	float* impulse_response;
	uint16_t length;
	float* circular_buffer;
	uint16_t first_element;
};

float update_filter(struct fir_filter* f, float new_value);


#endif /* __FIR_H__ */
