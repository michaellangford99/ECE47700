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
//#include "sensor_fusion.h"

struct PID yaw_pid;
struct PID pitch_pid;
struct PID roll_pid;

float motor_buffer[4][256];
struct fir_filter motor_filter[4];
float filtered_motor_output[4];
float motor_output[4] = {0.0f, 0.0f, 0.0f, 0.0f};
pwm_output_t pwm_output;

#define LED_PIN 13
#define LED_GPIO GPIOC

float max(float a, float b)
{
	if (a > b) return a;
	return b;
}

float min(float a, float b)
{
	if (a < b) return a;
	return b;
}

int main(void){

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//PC13 is the LED on test board
	//PA5 is the LED on the Nucleo
	LED_GPIO->MODER |= 0x1 << (LED_PIN * 2);
	LED_GPIO->MODER &= ~(0x2 << (LED_PIN * 2));

	LED_GPIO->ODR |= 0x1 << LED_PIN;

	init_USB_USART();

	//(print this within USB_USART startup so it's before the USB_USART config info
	//printf("\n-----------------------------------------\n");
	//printf("ECE477 STM32F446RET6 Flight Controller V0\n");
	//printf("-----------------------------------------\n\n");

	init_SYSTICK();

	init_RX_USART();
	init_PI_USART();

	init_PWM();
	//init_motors();

	init_I2C();
	//init_TMF8801();

	init_SPI1();
	init_LSM6DS3();
	init_MPU6500();

	calibrate_LSM6DS3();
	calibrate_MPU6500();

	init_PID(&yaw_pid);
	init_PID(&pitch_pid);
	init_PID(&roll_pid);

	set_PID_constants(&yaw_pid, 	0.0000f, 0.00f, 00.0f);
	set_PID_constants(&pitch_pid, 	0.0030f, 0.00f, 00.1f);
	set_PID_constants(&roll_pid, 	0.0030f, 0.00f, 00.1f);

	for (int i = 0; i < 4; i++)
	{
		motor_filter[i].impulse_response = hanning_64;
		motor_filter[i].length = 64;
		motor_filter[i].first_element = 0;
		motor_filter[i].circular_buffer = motor_buffer[i];
	}

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
	int d = 0;
	int m = 0;
	for(;;) {

		LED_GPIO->ODR ^= 0x1 << LED_PIN;

		update_LSM6DS3();
		update_MPU6500();

		float rx_throttle = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_THROTTLE);
		float rx_yaw 	  = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_YAW)   - 0.5f;
		float rx_pitch 	  = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_PITCH) - 0.5f;
		float rx_roll 	  = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_ROLL)  - 0.5f;

		float yaw_pid_response 	 = update_PID(&yaw_pid,   0/*gyro_angle_z*/, rx_yaw);
		float pitch_pid_response = update_PID(&pitch_pid, lsm6dsx_data.compl_pitch,  rx_pitch*30);
		float roll_pid_response  = update_PID(&roll_pid,  lsm6dsx_data.compl_roll,   rx_roll*30);

		//clamp PID outputs (depending on throttle?)
		float max_response = 0.2f;
		yaw_pid_response =   max(-max_response, yaw_pid_response);
		pitch_pid_response = max(-max_response, pitch_pid_response);
		roll_pid_response =  max(-max_response, roll_pid_response);
		
		yaw_pid_response =   min(max_response, yaw_pid_response);
		pitch_pid_response = min(max_response, pitch_pid_response);
		roll_pid_response =  min(max_response, roll_pid_response);

		motor_output[0] = rx_throttle;
		motor_output[1] = rx_throttle;
		motor_output[2] = rx_throttle;
		motor_output[3] = rx_throttle;

		if (rx_throttle > 0.05f)
		{
			motor_output[0] += yaw_pid_response;
			motor_output[1] += -yaw_pid_response;
			motor_output[2] += -yaw_pid_response;
			motor_output[3] += yaw_pid_response;

			motor_output[0] += pitch_pid_response;
			motor_output[1] += pitch_pid_response;
			motor_output[2] += -pitch_pid_response;
			motor_output[3] += -pitch_pid_response;

			motor_output[0] += -roll_pid_response;
			motor_output[1] += roll_pid_response;
			motor_output[2] += -roll_pid_response;
			motor_output[3] += roll_pid_response;
		}
		else
			lsm6dsx_data.gyro_angle_z = 0;

		//clamp motor values

		motor_output[0] = max(0.0f, motor_output[0]);
		motor_output[1] = max(0.0f, motor_output[1]);
		motor_output[2] = max(0.0f, motor_output[2]);
		motor_output[3] = max(0.0f, motor_output[3]);

		motor_output[0] = min(1.0f, motor_output[0]);
		motor_output[1] = min(1.0f, motor_output[1]);
		motor_output[2] = min(1.0f, motor_output[2]);
		motor_output[3] = min(1.0f, motor_output[3]);

		for (int i = 0; i < 4; i++)
		{
			shift_filter(&(motor_filter[i]), &(motor_output[i]), 1);
		}

		m++;
		if (m == 8)
		{
			m = 0;

			for (int i = 0; i < 4; i++)
			{
				filtered_motor_output[i] = compute_filter(&(motor_filter[i]));
			}

			pwm_output.duty_cycle_ch0 = (uint16_t)(filtered_motor_output[0]*65535.0f);
			pwm_output.duty_cycle_ch1 = (uint16_t)(filtered_motor_output[1]*65535.0f);
			pwm_output.duty_cycle_ch2 = (uint16_t)(filtered_motor_output[2]*65535.0f);
			pwm_output.duty_cycle_ch3 = (uint16_t)(filtered_motor_output[3]*65535.0f);

			set_PWM_duty_cycle(pwm_output);
		}

		d++;
		if (d > 50)
		{
			d = 0;

			//printf("%d,\t", s);
			//printf("%d,\t", RX_USART_get_channels()->ELRS_THROTTLE);
			//printf("%d,\t", RX_USART_get_channels()->ELRS_YAW);
			//printf("%d,\t", RX_USART_get_channels()->ELRS_PITCH);
			//printf("%d,\t", RX_USART_get_channels()->ELRS_ROLL);
			/*printf("%f,\t", rx_throttle);
			printf("%f,\t", rx_yaw);
			printf("%f,\t", rx_pitch);
			printf("%f,\t", rx_roll);
			printf("%f,\t", yaw_pid_response);
			printf("%f,\t", pitch_pid_response);
			printf("%f,\t", roll_pid_response);
			printf("%f,\t", filtered_motor_output[0]);
			printf("%f,\t", filtered_motor_output[1]);
			printf("%f,\t", filtered_motor_output[2]);
			printf("%f,\t", filtered_motor_output[3]);*/
			printf("%d,\t", pwm_output.duty_cycle_ch0);
			printf("%d,\t", pwm_output.duty_cycle_ch1);
			printf("%d,\t", pwm_output.duty_cycle_ch2);
			printf("%d,\t", pwm_output.duty_cycle_ch3);
			printf("%f,\t", lsm6dsx_data.compl_pitch);
			printf("%f,\n", lsm6dsx_data.compl_roll);

			//printf("%d,\t", RX_USART_get_channels()->ch4);
			//printf("%d,\t", RX_USART_get_channels()->ch5);
			//printf("%d,\t", RX_USART_get_channels()->ch6);
			//printf("%d,\t", RX_USART_get_channels()->ch7);
		}

		//char chr = __io_getchar();
		//printf("You entered %c.", chr);

		//printf("Enter your name: %d\r\n", 42069);
	}
}
