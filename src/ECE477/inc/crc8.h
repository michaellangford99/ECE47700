#ifndef __CRC8_H__
#define __CRC8_H__

void init_crc8(uint8_t poly);
uint8_t calc_crc(uint8_t *data, uint8_t len);

uint8_t crc_lut[256];

#endif /* __CRC8_H__ */
