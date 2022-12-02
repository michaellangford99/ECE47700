#ifndef __PI_USART_H__
#define __PI_USART_H__

typedef struct {
	uint8_t 		header;
	uint8_t 		camera_status;
	uint16_t 		lidar_reading[4];
	//uint8_t		zero_pad[2];
	float			command_yaw;
	float			command_pitch;
	float			command_roll;
	uint8_t 		ending;
} /*__attribute__ ((packed))*/ pi_packet_t;

extern pi_packet_t saved_pi_packet;

void init_PI_USART(void);
void test_PI_USART();
void update_PI_USART();

#endif /* __PI_USART_H__ */
