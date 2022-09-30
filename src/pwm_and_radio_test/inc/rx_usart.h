#ifndef __RX_USART_H__
#define __RX_USART_H__

typedef struct
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
} __attribute__ ((packed)) crsf_channels_t;

typedef struct
{
	uint8_t start_code;
	uint8_t packet_length;
	uint8_t address;
	crsf_channels_t channels;
	uint8_t crc;
} __attribute__ ((packed)) crsf_packet_t;


crsf_channels_t saved_channel_data;


void init_RX_USART(void);

#endif /* __RX_USART_H__ */
