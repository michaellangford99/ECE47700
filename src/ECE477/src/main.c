/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include <stdio.h>

#include "system.h"
#include "fifo.h"
#include "tty.h"
#include "rx_usart.h"
#include "usb_usart.h"
#include "pi_usart.h"
#include "pwm.h"
#include "i2c.h"
#include "motors.h"
#include "lsm6ds3.h"
//#include "sensor_fusion.h"


#define LED_PIN 5//13
#define LED_GPIO GPIOA//GPIOC

int main(void){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//PC13 is the LED on test board
	//PA5 is the LED on the Nucleo
	LED_GPIO->MODER |= 0x1 << (LED_PIN * 2);
	LED_GPIO->MODER &= ~(0x2 << (LED_PIN * 2));

	LED_GPIO->ODR |= 0x1 << LED_PIN;

	//init_SYSTICK();

	init_USB_USART();
	init_RX_USART();
	//init_PI_USART();

	init_PWM();
	//init_motors();

	//init_I2C();
	//init_TMF8801();

	//init_SPI1();
	//init_LSM6DS3();
	//test_LSM6DS3();

	//main loop:
	for(;;)
	{
		break;
		//On medium update rate
			//radio updates via DMA
			//grab channel values
			//communicate IMU and other log data to PI, and read in any new commands
			//update safety code with the unlocking and the motor stop channels
			//monitor autonomous switch

		//very slow update rate
			//grab I2C LIDAR distance to ground.
				//if we want to be fancy, generate distance to 
				//ground as dot product of distance reading and 
				//down vector (generated via pitch, roll unit vectors)
		
		//as fast as possible
		//determine time delta
		//	(you should find what the 
		//	 system can do and then force it)
		//read gyro and accelerometer
		//(add magnetometer update later)
			//estimate yaw from gyro and magnetometer
		//lpf gyro and acc data if necessary
		//compute atan2 of acc data to find yaw, pitch, roll
		//compute complementary filter of gyro rate, time delta, and acc angles

		//based on autonomous mode / radio control, select target yaw, pitch
		// roll, and height above ground from either PI data or radio
			//for throttle, it is either autonomously controlled via LIDAR and 
			//a pi height setpoint - or it uses no PID and passes the throttle
			//value from the radio directly to the motors

		//pass yaw, pitch, roll, and throttle (if needed) to PID loops
		//update PIDs

		//translate PID outputs into motor values
			//linear combination of the responses, coefficients based on the motor

		//send updated motor commands to motors
			//motor driver will internally do any necessary low pass filter

		//medium update rate, or as needed
			//dump log data out over USB
	}


	for(;;) {

		for (volatile int i = 9900; i > 0; i--)
		{
			__asm("NOP");
		}
		//LED_GPIO->ODR ^= 0x1 << LED_PIN;

		pwm_output_t pwm_output;

		pwm_output.duty_cycle_ch0 = (((2^16) - 1)*(uint32_t)(RX_USART_get_channels()->ch0 - CRSF_CHANNEL_VALUE_MIN))/(CRSF_CHANNEL_VALUE_MAX-CRSF_CHANNEL_VALUE_MIN);
		pwm_output.duty_cycle_ch1 = (((2^16) - 1)*(uint32_t)(RX_USART_get_channels()->ch1 - CRSF_CHANNEL_VALUE_MIN))/(CRSF_CHANNEL_VALUE_MAX-CRSF_CHANNEL_VALUE_MIN);
		pwm_output.duty_cycle_ch2 = (((2^16) - 1)*(uint32_t)(RX_USART_get_channels()->ch2 - CRSF_CHANNEL_VALUE_MIN))/(CRSF_CHANNEL_VALUE_MAX-CRSF_CHANNEL_VALUE_MIN);
		pwm_output.duty_cycle_ch3 = (((2^16) - 1)*(uint32_t)(RX_USART_get_channels()->ch3 - CRSF_CHANNEL_VALUE_MIN))/(CRSF_CHANNEL_VALUE_MAX-CRSF_CHANNEL_VALUE_MIN);

		set_PWM_duty_cycle(pwm_output);

		printf("%d,\t", RX_USART_get_channels()->ch0);
		printf("%d,\t", RX_USART_get_channels()->ch1);
		printf("%d,\t", RX_USART_get_channels()->ch2);
		printf("%d,\t", RX_USART_get_channels()->ch3);
		printf("%d,\t", RX_USART_get_channels()->ch4);
		printf("%d,\t", RX_USART_get_channels()->ch5);
		printf("%d,\t", RX_USART_get_channels()->ch6);
		printf("%d\n",  RX_USART_get_channels()->ch7);

		//char chr = __io_getchar();
		//printf("You entered %c.", chr);

		//printf("Enter your name: %d\r\n", 42069);
	}
}
