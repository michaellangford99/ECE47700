#include "stm32f4xx.h"
#include <stdio.h>

#include "system.h"
#include "fifo.h"
#include "tty.h"
#include "rx_usart.h"
#include "usb_usart.h"
#include "pi_usart.h"
#include "systick.h"
#include "pwm.h"
#include "spi.h"
#include "i2c.h"
#include "motors.h"
#include "lsm6ds3.h"
#include "mpu6500.h"
#include "pid.h"
#include "math.h"
#include "fir.h"
#include "safety.h"

#define ARM_THROTTLE_MAX	0.1f
#define ARM_YAW_MAX			0.1f
#define ARM_PITCH_MAX		0.1f
#define ARM_ROLL_MIN		0.9f

#define ARM_CODE_BYTE_0 4
#define ARM_CODE_BYTE_1 7
#define ARM_CODE_BYTE_2 7
#define ARM_CODE_BYTE_3 8

#define LED_PIN 13
#define LED_GPIO GPIOC

#define SAFE_FLASH_PERIOD  1000
#define ARMED_FLASH_PERIOD 100

int led_flash_count;
int led_flash_period = SAFE_FLASH_PERIOD;

//for security, use 4 bytes.. all must change to the arm code to arm the drone
uint8_t ARMED[4] = {0, 0, 0, 0};

uint8_t check_arm_code()
{
	if ((ARMED[0] == ARM_CODE_BYTE_0) &&
		(ARMED[1] == ARM_CODE_BYTE_1) &&
		(ARMED[2] == ARM_CODE_BYTE_2) &&
		(ARMED[3] == ARM_CODE_BYTE_3))
		return 1;
	return 0;
}

void set_arm_code()
{
	ARMED[0] = ARM_CODE_BYTE_0;
	ARMED[1] = ARM_CODE_BYTE_1;
	ARMED[2] = ARM_CODE_BYTE_2;
	ARMED[3] = ARM_CODE_BYTE_3;

	led_flash_period = ARMED_FLASH_PERIOD;
}

void clear_arm_code()
{
	ARMED[0] = 0;
	ARMED[1] = 0;
	ARMED[2] = 0;
	ARMED[3] = 0;

	led_flash_period = SAFE_FLASH_PERIOD;
}

void safety_LED()
{
	led_flash_count++;
	if (led_flash_count >= led_flash_period)
	{
		led_flash_count = 0;
		LED_GPIO->ODR ^= 0x1 << LED_PIN;
	}
}

int safety_update_count;
#define SAFETY_UPDATE_PERIOD 10

uint8_t safety()
{
	//only actually do the sequence periodically, usually return the saved arm status
	safety_update_count++;
	if (safety_update_count > SAFETY_UPDATE_PERIOD)
	{
		safety_update_count = 0;
	}
	else
	{
		return check_arm_code();
	}

	//if the radio doesn't think it has a valid signal, abort
	if (radio_signal_status() == 0)
	{
		clear_arm_code();
		return 0;
	}

	//first check that all aux signals are at least at or above the min and below or at the max
	uint8_t active_aux_signals = 0;

	//check that signals are in range
	if ((RX_USART_get_channels()->ELRS_SA >= ELRS_AUX_MIN) && (RX_USART_get_channels()->ELRS_SA <= ELRS_AUX_MAX)) active_aux_signals++;
	if ((RX_USART_get_channels()->ELRS_SB >= ELRS_AUX_MIN) && (RX_USART_get_channels()->ELRS_SB <= ELRS_AUX_MAX)) active_aux_signals++;
	if ((RX_USART_get_channels()->ELRS_SC >= ELRS_AUX_MIN) && (RX_USART_get_channels()->ELRS_SC <= ELRS_AUX_MAX)) active_aux_signals++;
	if ((RX_USART_get_channels()->ELRS_SD >= ELRS_AUX_MIN) && (RX_USART_get_channels()->ELRS_SD <= ELRS_AUX_MAX)) active_aux_signals++;

	//if all 4 aren't in range, abort
	if (active_aux_signals != 4)
	{
		clear_arm_code();
		return 0;
	}

	//check that arming aux switch is in armed position
	if (RX_USART_get_channels()->ELRS_ARM_CHANNEL < ELRS_AUX_MAX)
	{
		clear_arm_code();
		return 0;
	}

	float rx_throttle = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_THROTTLE);
	float rx_yaw = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_YAW);
	float rx_pitch = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_PITCH);
	float rx_roll = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_ROLL);

	//if not yet armed, check for arm condition (both sticks down and to the sides)
	if (!check_arm_code())
		if (rx_throttle < ARM_THROTTLE_MAX)
			if (rx_yaw < ARM_YAW_MAX)
				if (rx_pitch < ARM_PITCH_MAX)
					if (rx_roll > ARM_ROLL_MIN)
					{
						set_arm_code();
					}

	return check_arm_code();
}

