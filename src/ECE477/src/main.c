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

#include "fifo.h"
#include "tty.h"
#include "rx_usart.h"
#include "usb_usart.h"
#include "pwm.h"

#define LED_PIN 13
#define LED_GPIO GPIOC

int main(void){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//PC13 is the LED on test board
	//PA5 is the LED on the Nucleo
	LED_GPIO->MODER |= 0x1 << (LED_PIN * 2);
	LED_GPIO->MODER &= ~(0x2 << (LED_PIN * 2));

	LED_GPIO->ODR |= 0x1 << LED_PIN;

	init_SYSTICK();

	init_USB_USART();
	init_RX_USART();
	init_PI_USART();

	init_PWM();
	//init_motors();

	init_I2C();
	//init_TMF8801();

	init_SPI1();
	init_LSM6DS3();
	test_LSM6DS3();

	//main loop:
	for(;;)
	{
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

		for (volatile int i = 99; i > 0; i--)
		{
			__asm("NOP");
		}
		LED_GPIO->ODR ^= 0x1 << LED_PIN;

		printf("%d, \t", set_PWM_duty_cycle());

		printf("%d,\t", saved_channel_data.ch0);
		printf("%d,\t", saved_channel_data.ch1);
		printf("%d,\t", saved_channel_data.ch2);
		printf("%d,\t", saved_channel_data.ch3);
		printf("%d,\t", saved_channel_data.ch4);
		printf("%d,\t", saved_channel_data.ch5);
		printf("%d,\t", saved_channel_data.ch6);
		printf("%d\n",  saved_channel_data.ch7);

		//char chr = __io_getchar();
		//printf("You entered %c.", chr);

		//printf("Enter your name: %d\r\n", 42069);
	}
}
