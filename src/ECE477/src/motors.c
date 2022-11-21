#include "stm32f4xx.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "system.h"
#include "pwm.h"
#include "motors.h"

//TODO
/*

init motors

update_safety
    pass in pertinent radio axes
    convert from radio range to 0-65535


set motors

convert motor range into range usuable by pwm code

(uint16_t)((((uint32_t)65535)*(uint32_t)(RX_USART_get_channels()->ch2 - CRSF_CHANNEL_VALUE_MIN))/(uint32_t)(CRSF_CHANNEL_VALUE_MAX-CRSF_CHANNEL_VALUE_MIN));

*/
