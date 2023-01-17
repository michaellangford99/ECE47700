#include "stm32f4xx.h"

#include <stdio.h>
#include <stdint.h>
#include "crc8.h"

///
/// Taken and modified from https://github.com/mikeneiderhauser/CRSFJoystick/blob/master/Sketchbook/libraries/crc8/crc8.cpp
///

void init_crc8(uint8_t poly)
{
    for (int idx=0; idx<256; ++idx)
    {
        uint8_t crc = idx;
        for (int shift=0; shift<8; ++shift)
        {
            crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
        }
        crc_lut[idx] = crc & 0xff;
    }
}

uint8_t calc_crc(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc = crc_lut[crc ^ *data++];
    }
    return crc;
}
