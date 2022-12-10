/**
  ******************************************************************************
  * @file    main.c
  * @author  Michael Langford
  * @date    06-December-2022
  * @brief   Flight controller main control loop
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
#include "safety.h"
#include "SparkFun_TMF8801_Arduino_Library.h"
#include "SparkFun_TMF8801_Constants.h"
#include "SparkFun_TMF8801_IO.h"
//#include "sensor_fusion.h"

struct PID yaw_pid;
struct PID pitch_pid;
struct PID roll_pid;

float auto_throttle_response;
float auto_throttle_setpoint = 220.0f;
float auto_throttle_center_point;
struct PID auto_throttle_pid;

float filt_yaw_command = 0;
float filt_pitch_command = 0;
float filt_roll_command = 0;

float motor_buffer[4][256];
struct fir_filter motor_filter[4];
float filtered_motor_output[4];
float motor_output[4] = {0.0f, 0.0f, 0.0f, 0.0f};
pwm_output_t pwm_output;

TMF8801_t device_descrip;

//timestamps
static float last_time_2;
static float current_time_2;
static float last_time;
static float current_time;

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
	init_TMF8801(&device_descrip);

	init_SPI1();
	init_LSM6DS3();
	init_MPU6500();

	//wait for foam to stabilize
	 wait(2.0f);

	calibrate_LSM6DS3();
	calibrate_MPU6500();

	init_PID(&yaw_pid);
	init_PID(&pitch_pid);
	init_PID(&roll_pid);
	init_PID(&auto_throttle_pid);

	set_PID_constants(&yaw_pid, 	0.0015f, 0.00f, 01.3f);
	set_PID_constants(&pitch_pid, 	0.0010f, 0.00f, 01.0);
	set_PID_constants(&roll_pid, 	0.0010f, 0.00f, 01.0f);

	set_PID_constants(&auto_throttle_pid, 	0.00005f, 0.00000f, 0.1f);
	auto_throttle_pid.maxP=0.1f;
	auto_throttle_pid.maxI=0.05f;

	for (int i = 0; i < 4; i++)
	{
		motor_filter[i].impulse_response = hanning_8;
		motor_filter[i].length = 8;
		motor_filter[i].first_element = 0;
		motor_filter[i].circular_buffer = motor_buffer[i];
	}

	//main loop:
	int d = 0;
	int m = 0;
	for(;;) {

		safety_LED();

		last_time = current_time;
		current_time = ftime();

		update_LSM6DS3();
		update_MPU6500();
		update_PI_USART();
		update_TMF8801(&device_descrip);

		float rx_throttle = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_THROTTLE);
		float rx_yaw 	  = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_YAW)   - 0.5f;
		float rx_pitch 	  = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_PITCH) - 0.5f;
		float rx_roll 	  = RX_USART_convert_channel_to_unit_range(RX_USART_get_channels()->ELRS_ROLL)  - 0.5f;

		if (RX_USART_get_channels()->ELRS_SD < ELRS_AUX_MAX)
		{
			rx_throttle *= 0.3f;
		}


		//check for autonomous mode (TODO: check LiDAR timeout)
		if (RX_USART_get_channels()->ELRS_SC >= ELRS_AUX_MID)
		{
			//clearly can't use a D in this PID due to switching on and off
			float dist_measurement = get_distance_TMF8801(&device_descrip);
			auto_throttle_response = update_PID(&auto_throttle_pid, (float)dist_measurement, auto_throttle_setpoint);

			auto_throttle_response = min(0.05f, auto_throttle_response);
			auto_throttle_response = max(-0.05f, auto_throttle_response);

			rx_throttle = auto_throttle_center_point + auto_throttle_response;
			rx_throttle = (rx_throttle < 0.05f) ? 0.05f : rx_throttle;

			//if in FULLY auto mode, use the incoming pitch and roll commands from the Pi
			if (RX_USART_get_channels()->ELRS_SC == ELRS_AUX_MAX)
			{
				rx_yaw	 = saved_pi_packet.command_yaw;
				rx_pitch = saved_pi_packet.command_pitch;
				rx_roll  = saved_pi_packet.command_roll;
			}
		}
		else
		{
			//save last manual-mode throttle
			auto_throttle_center_point = rx_throttle;
			auto_throttle_setpoint = get_distance_TMF8801(&device_descrip);
		}

		filt_yaw_command  += -rx_yaw/40.0f;
		filt_pitch_command = 0.003f*rx_pitch + 0.997f*filt_pitch_command;
		filt_roll_command  = 0.003f*rx_roll  + 0.997f*filt_roll_command;

		float yaw_pid_response 	 = update_PID(&yaw_pid,   lsm6dsx_data.gyro_angle_z, filt_yaw_command);
		float pitch_pid_response = update_PID(&pitch_pid, lsm6dsx_data.compl_pitch,  filt_pitch_command*40);
		float roll_pid_response  = update_PID(&roll_pid,  lsm6dsx_data.compl_roll,   -filt_roll_command*40);

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

		if (rx_throttle > 0.01f)
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
		{
			clear_yaw();
			filt_yaw_command = 0.0f;
		}

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

		//decimate and low pass motor values
		//(FIR filter so able to move to output side of decimator)
		m++;
		if (m == 4)
		{
			m = 0;

			for (int i = 0; i < 4; i++)
			{
				filtered_motor_output[i] = compute_filter(&(motor_filter[i]));
			}

			if (safety())
			{
				uint32_t m0 = (uint32_t)(filtered_motor_output[0]*65535.0f);
				uint32_t m1 = (uint32_t)(filtered_motor_output[1]*65535.0f);
				uint32_t m2 = (uint32_t)(filtered_motor_output[2]*65535.0f);
				uint32_t m3 = (uint32_t)(filtered_motor_output[3]*65535.0f);


				pwm_output.duty_cycle_ch0 = m0>65535 ? 65535 : (uint16_t)m0;
				pwm_output.duty_cycle_ch1 = m1>65535 ? 65535 : (uint16_t)m1;
				pwm_output.duty_cycle_ch2 = m2>65535 ? 65535 : (uint16_t)m2;
				pwm_output.duty_cycle_ch3 = m3>65535 ? 65535 : (uint16_t)m3;

				set_PWM_duty_cycle(pwm_output);
			}
			else
			{
				pwm_output.duty_cycle_ch0 = 0;
				pwm_output.duty_cycle_ch1 = 0;
				pwm_output.duty_cycle_ch2 = 0;
				pwm_output.duty_cycle_ch3 = 0;

				set_PWM_duty_cycle(pwm_output);
			}
		}

		//notify systick that main routine is still alive
		systick_clear_watchdog();

		d++;
		if (d > 50)
		{
			d = 0;

			//printf("%d,\t", s);
			//printf("%d,\t", RX_USART_get_channels()->ch4);
			//printf("%d,\t", RX_USART_get_channels()->ch5);
			//printf("%d,\t", RX_USART_get_channels()->ch6);
			//printf("%d,\t", RX_USART_get_channels()->ch7);
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
			//printf("%.2f,\t", 1.0f/(current_time - last_time));
			//printf("%.2f,\t", 1.0f/(current_time_2 - last_time_2));
			/*printf("%d,\t", check_arm_code());
			printf("%d,\t", saved_pi_packet.camera_status);
			printf("%d,\t", saved_pi_packet.lidar_reading[0]);
			printf("%d,\t", saved_pi_packet.lidar_reading[1]);
			printf("%d,\t", saved_pi_packet.lidar_reading[2]);
			printf("%d,\t", saved_pi_packet.lidar_reading[3]);
			printf("%.2f,\t", saved_pi_packet.command_yaw);
			printf("%.2f,\t", saved_pi_packet.command_pitch);
			printf("%.2f,\t", saved_pi_packet.command_roll);
			printf("%d,\t", pwm_output.duty_cycle_ch0);
			printf("%d,\t", pwm_output.duty_cycle_ch1);
			printf("%d,\t", pwm_output.duty_cycle_ch2);
			printf("%d,\t", pwm_output.duty_cycle_ch3);
			printf("%.3f,\t", lsm6dsx_data.gyro_angle_z);
			printf("%.3f,\t", lsm6dsx_data.compl_pitch);
			printf("%.3f,\t", lsm6dsx_data.compl_roll);*/

			printf("%.3f,\t", get_distance_TMF8801(&device_descrip));

			printf("%.3f,\t", rx_throttle);
			printf("%.3f,\t", auto_throttle_center_point);
			printf("%.3f,\t", auto_throttle_response);

			printf("%.3f,\t", filt_yaw_command);
			printf("%.3f,\t", filt_pitch_command*40);
			printf("%.3f,\t", filt_roll_command*40);

			//read_distance(&device_descrip);
			//printf("%d,\t", RX_USART_get_channels()->ch4);
			//printf("%d,\t", RX_USART_get_channels()->ch5);
			//printf("%d,\t", RX_USART_get_channels()->ch6);
			//printf("%d,\t", RX_USART_get_channels()->ch7);
			printf("\n");
		}

		last_time_2 = current_time_2;
		current_time_2 = ftime();
	}

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
}
