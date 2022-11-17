/**
  ******************************************************************************
  * @file    lsm6ds3.c
  * @author  OWen Mandel, Michael Langford
  * @version V1.0
  * @date    11-November-2022
  * @brief   LSM6DS3 6DoF IMU driver.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "string.h"
#include "spi.h"
#include "lsm6ds3.h"
#include "systick.h"

int16_t gyro_raw_x;
int16_t gyro_raw_y;
int16_t gyro_raw_z;

int16_t accel_raw_x;
int16_t accel_raw_y;
int16_t accel_raw_z;

//large data type to allow for very large integration time
int64_t gyro_cal_x;
int64_t gyro_cal_y;
int64_t gyro_cal_z;

int64_t accel_cal_x;
int64_t accel_cal_y;
int64_t accel_cal_z;

//in units of deg/s
float gyro_rate_x;
float gyro_rate_y;
float gyro_rate_z;

//in units of deg - this is not assisted by any other sensor data
float gyro_angle_x;
float gyro_angle_y;
float gyro_angle_z;

//in units of g's
float accel_x;
float accel_y;
float accel_z;

//timestamps
float last_time;
float current_time;

//first off, we want to figure out if we are doing fixed point or what
//2 g max means +/- 32768 covers the range +/- 2g
//1g = 9.81 m/s^2
//so a raw  number->
//converted_to_gs = (raw / 32768.0f)*(2.0f)*(9.81f)

//CHECK A_GAIN TABLE TO NOT HAVE TO DO ABOVE CALCULATION TO CONVERT TO G'S

//degrees per second
//first of all, systick needs to have high precision
//not sure what the range on this is.

void init_LSM6DS3(){
	
	//inital LSM6DSO parameters (see LSM6DSO reference manual (https://www.st.com/en/mems-and-sensors/lsm6dsl.html))
	uint8_t ctrl1 = CTRL1_DEF | ODR_XL2 | ODR_XL1;
	uint8_t ctrl2 = CTRL2_DEF | ODR_G3  | ODR_G1 | GYRO_FS_500DPS;
	uint8_t ctrl3 = CTRL3_DEF;
	uint8_t ctrl4 = CTRL4_DEF | I2C_disable;
	uint8_t ctrl5 = CTRL5_DEF;
	uint8_t ctrl6 = CTRL6_DEF;
	uint8_t ctrl7 = CTRL7_DEF;
	uint8_t ctrl8 = CTRL8_DEF;
	uint8_t ctrl9 = CTRL9_DEF;
	uint8_t ctrl10 = CTRL10_DEF;

	//ctrl register writing
	writeReg(CTRL1_XL, 	ctrl1);
	writeReg(CTRL2_G, 	ctrl2);
	writeReg(CTRL3_G, 	ctrl3);
	writeReg(CTRL4_G, 	ctrl4);
	writeReg(CTRL5_G, 	ctrl5);
	writeReg(CTRL6_G, 	ctrl6);
	writeReg(CTRL7_G, 	ctrl7);
	writeReg(CTRL8_XL, 	ctrl8);
	writeReg(CTRL9_XL, 	ctrl9);
	writeReg(CTRL10_C,	ctrl10);

	current_time = ftime();
	last_time = current_time;
}

void calibrate_LSM6DS3()
{
	gyro_cal_x = 0;
	gyro_cal_y = 0;
	gyro_cal_z = 0;

	for(int i = 0; i < 10000; i++){
		__asm("NOP");
	}

	for (int i = 0; i < CAL_LENGTH; i++)
	{
		//set precise sampling interval to prevent aliasing
		wait(1.0f/(6.67f*1000.0f));

		read_axes();
		gyro_cal_x += gyro_raw_x;
		gyro_cal_y += gyro_raw_y;
		gyro_cal_z += gyro_raw_z;
	}

	gyro_cal_x = gyro_cal_x / CAL_LENGTH;
	gyro_cal_y = gyro_cal_y / CAL_LENGTH;
	gyro_cal_z = gyro_cal_z / CAL_LENGTH;

	printf("LSM6DSO Calibration:\n\tgyro_cal_x: %d\n\tgyro_cal_y: %d\n\tgyro_cal_z: %d\n", gyro_cal_x, gyro_cal_y, gyro_cal_z);

	//intentionally stting delta time to zero
	current_time = ftime();
	last_time = current_time;
}

void read_axes(){
	uint8_t data_buf[12];
	for (int i = 0; i < 12; i++)
	{
		data_buf[i] = readReg(OUTX_L_G+i);
		data_buf[i] = readReg(OUTX_L_G+i);
	}

	gyro_raw_x  = ((int16_t)data_buf[0]  | ((int16_t)data_buf[1]  << 8));
	gyro_raw_y  = ((int16_t)data_buf[2]  | ((int16_t)data_buf[3]  << 8));
	gyro_raw_z  = ((int16_t)data_buf[4]  | ((int16_t)data_buf[5]  << 8));
	accel_raw_x = ((int16_t)data_buf[6]  | ((int16_t)data_buf[7]  << 8));
	accel_raw_y = ((int16_t)data_buf[8]  | ((int16_t)data_buf[9]  << 8));
	accel_raw_z = ((int16_t)data_buf[10] | ((int16_t)data_buf[11] << 8));
}

int d = 0;

void update_LSM6DS3(void){
   
	read_axes();

	last_time = current_time;
	current_time = ftime();

	gyro_rate_x = (gyro_raw_x - gyro_cal_x) * G_GAIN_500DPS;
	gyro_rate_y = (gyro_raw_y - gyro_cal_y) * G_GAIN_500DPS;
	gyro_rate_z = (gyro_raw_z - gyro_cal_z) * G_GAIN_500DPS;

	gyro_angle_x += gyro_rate_x * (current_time - last_time);
	gyro_angle_y += gyro_rate_y * (current_time - last_time);
	gyro_angle_z += gyro_rate_z * (current_time - last_time);

	d++;

	if (d > 100)
	{
		d = 0;
		printf("%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%d,\t%d,\t%d,\t%f\n", gyro_rate_x, gyro_rate_y, gyro_rate_z, gyro_angle_x, gyro_angle_y, gyro_angle_z, accel_raw_x, accel_raw_y, accel_raw_z, current_time - last_time);
		//printf("%d, %d, %d\n", gyro_raw_x - gyro_cal_x, gyro_raw_y - gyro_cal_y, gyro_raw_z - gyro_cal_z);
	}
	//printf("%d\n", accel_raw_x);
}
