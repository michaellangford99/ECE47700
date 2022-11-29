/**
  ******************************************************************************
  * @file    mpu6500.c
  * @author  Michael Langford
  * @version V1.0
  * @date    26-November-2022
  * @brief   MPU6500 6DoF IMU driver.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "system.h"
#include "string.h"
#include "spi.h"
#include "mpu6500.h"
#include "systick.h"
#include "math.h"
#include "fir.h"

Gyro_FS_Sel gyro_fs = _250DPS;
Gyro_Config gyro_config;
Accel_FS_Sel accel_fs = _4g;
Accel_Config accel_config;
//puts data in correct units after scale selector applied
const float GYRO_MULTIPLIER = 250.0f;

static int16_t gyro_raw_x;
static int16_t gyro_raw_y;
static int16_t gyro_raw_z;

static int16_t accel_raw_x;
static int16_t accel_raw_y;
static int16_t accel_raw_z;

//large data type to allow for very large integration time
static int64_t gyro_cal_x;
static int64_t gyro_cal_y;
static int64_t gyro_cal_z;

static int64_t accel_cal_x;
static int64_t accel_cal_y;
static int64_t accel_cal_z;

//in units of deg/s
static float gyro_rate_x;
static float gyro_rate_y;
static float gyro_rate_z;

//in units of deg - this is not assisted by any other sensor data
static float gyro_angle_x;
static float gyro_angle_y;
static float gyro_angle_z;

//in units of g's
static float accel_x;
static float accel_y;
static float accel_z;

#define TOTAL_ACC_DECIMATION 10
static int total_acceleration_decimation_index;
static struct fir_filter total_acceleration_filter;
static float total_acceleration[64];
static float total_acceleration_filtered;

static float accel_pitch;
static float accel_roll;

static float compl_pitch;
static float compl_roll;

imu_data_t mpu6500_data;

//timestamps
static float last_time;
static float current_time;

//first off, we want to figure out if we are doing fixed point or what
//2 g max means +/- 32768 covers the range +/- 2g
//1g = 9.81 m/s^2
//so a raw  number->
//converted_to_gs = (raw / 32768.0f)*(2.0f)*(9.81f)

//CHECK A_GAIN TABLE TO NOT HAVE TO DO ABOVE CALCULATION TO CONVERT TO G'S

//degrees per second
//first of all, systick needs to have high precision
//not sure what the range on this is.

void init_MPU6500(){
	
	//ctrl register writing
	select_SPI(SELECT_MPU_SPI);

	writeReg(MPU6500_CONFIG, 	0);

    accel_config.bits.ACCEL_FS_SEL = accel_fs;
	writeReg(MPU6500_ACCEL_CONFIG, accel_config.raw);

	gyro_config.bits.GYRO_FS_SEL = gyro_fs;
	writeReg(MPU6500_GYRO_CONFIG, accel_config.raw);

	//init test filter:
	total_acceleration_filter.circular_buffer = total_acceleration;
	total_acceleration_filter.first_element = 0;
	total_acceleration_filter.impulse_response = hanning_64;
	total_acceleration_filter.length = 64;

	current_time = ftime();
	last_time = current_time;
}

void calibrate_MPU6500()
{
	gyro_cal_x = 0;
	gyro_cal_y = 0;
	gyro_cal_z = 0;

	accel_cal_x = 0;
	accel_cal_y = 0;
	accel_cal_z = 0;

	for(int i = 0; i < 10000; i++){
		__asm("NOP");
	}

	for (int i = 0; i < CAL_LENGTH; i++)
	{
		//set precise sampling interval to prevent aliasing
		wait(1.0f/(6.67f*1000.0f));

		read_axes_MPU6500();
		gyro_cal_x += gyro_raw_x;
		gyro_cal_y += gyro_raw_y;
		gyro_cal_z += gyro_raw_z;

		accel_cal_x += accel_raw_x;
		accel_cal_y += accel_raw_y;
		accel_cal_z += accel_raw_z - A_RAW_1G; //Z axis experiences gravity, this cals it to 1G
	}

	gyro_cal_x = gyro_cal_x / CAL_LENGTH;
	gyro_cal_y = gyro_cal_y / CAL_LENGTH;
	gyro_cal_z = gyro_cal_z / CAL_LENGTH;

	accel_cal_x = accel_cal_x / CAL_LENGTH;
	accel_cal_y = accel_cal_y / CAL_LENGTH;
	accel_cal_z = accel_cal_z / CAL_LENGTH;

	printf("MPU6500 Calibration:\n\tgyro_cal_x: %d\n\tgyro_cal_y: %d\n\tgyro_cal_z: %d\n", gyro_cal_x, gyro_cal_y, gyro_cal_z);

	//intentionally stting delta time to zero
	current_time = ftime();
	last_time = current_time;
}

void read_axes_MPU6500(){
	uint8_t data_buf[12];
	
	select_SPI(SELECT_MPU_SPI);
	for (int i = 0; i < 6; i++)
	{
		data_buf[i] = readReg(MPU6500_GYRO_XOUT_H+i);
		data_buf[i] = readReg(MPU6500_GYRO_XOUT_H+i);
	}
    
    for (int i = 0; i < 6; i++)
	{
		data_buf[6+i] = readReg(MPU6500_ACCEL_XOUT_H+i);
		data_buf[6+i] = readReg(MPU6500_ACCEL_XOUT_H+i);
	}

	gyro_raw_x  = ((int16_t)data_buf[1]  | ((int16_t)data_buf[0]  << 8));
	gyro_raw_y  = ((int16_t)data_buf[3]  | ((int16_t)data_buf[2]  << 8));
	gyro_raw_z  = ((int16_t)data_buf[5]  | ((int16_t)data_buf[4]  << 8));
	accel_raw_x = ((int16_t)data_buf[7]  | ((int16_t)data_buf[6]  << 8));
	accel_raw_y = ((int16_t)data_buf[9]  | ((int16_t)data_buf[8]  << 8));
	accel_raw_z = ((int16_t)data_buf[11] | ((int16_t)data_buf[10] << 8));
}

int d_mpu = 0;

void update_MPU6500(void){
   
	read_axes_MPU6500();

	last_time = current_time;
	current_time = ftime();

	gyro_rate_x = (gyro_raw_x - gyro_cal_x) * G_GAIN_500DPS;
	gyro_rate_y = (gyro_raw_y - gyro_cal_y) * G_GAIN_500DPS;
	gyro_rate_z = (gyro_raw_z - gyro_cal_z) * G_GAIN_500DPS;

	gyro_angle_x += gyro_rate_x * (current_time - last_time);
	gyro_angle_y += gyro_rate_y * (current_time - last_time);
	gyro_angle_z += gyro_rate_z * (current_time - last_time);

	accel_x = (accel_raw_x - accel_cal_x) * A_GAIN_4G;
	accel_y = (accel_raw_y - accel_cal_y) * A_GAIN_4G;
	accel_z = (accel_raw_z - accel_cal_z) * A_GAIN_4G;

	accel_pitch = atan2(accel_x, sqrt(accel_z*accel_z + accel_y*accel_y)) * (180.0f/3.1415963f) * 0.01f + 0.99f*accel_pitch;
	accel_roll = atan2(accel_y, sqrt(accel_z*accel_z + accel_x*accel_x)) * (180.0f/3.1415963f) * 0.01f + 0.99f*accel_roll;

	//total_acceleration_decimation_index++;
	//if (total_acceleration_decimation_index == TOTAL_ACC_DECIMATION)
	//{
	//	total_acceleration_decimation_index = 0;
		total_acceleration_filtered = update_filter(&total_acceleration_filter, sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z));
	//}
	//trust of the accel
	#define ALPHA_0 0.06f//0.61f

	float alpha_correction = pow(total_acceleration_filtered-1.0f, 2.0f);
	//bigger this is, less it should trust accel
	alpha_correction=-alpha_correction*10.0f;
	alpha_correction = (alpha_correction < -ALPHA_0) ? -ALPHA_0 : alpha_correction;

#define ALPHA (ALPHA_0/* + alpha_correction*/)


	compl_pitch += (1.0f-ALPHA)*(-gyro_rate_y*2.0f)*(current_time - last_time) + ALPHA * (accel_pitch-compl_pitch);
	compl_roll +=  (1.0f-ALPHA)*(gyro_rate_x*2.0f)*(current_time - last_time) + ALPHA * (accel_roll-compl_roll);

	d_mpu++;

	if (d_mpu > 100)
	{
		d_mpu = 0;

		//printf("%f,\t%f,\t%f,\t%f,\n", accel_x, accel_y, accel_z, current_time - last_time);

		//printf("%f,\t%f,\t%f,\t%f\n", compl_pitch, compl_roll, (current_time - last_time), alpha_correction);

		//printf("%f,\t%f,\t%f,\t%f\n", gyro_angle_x, gyro_angle_y, accel_pitch * 180.0f/3.1415963f, accel_roll * 180.0f/3.1415963f);

		//printf("%f,\t%f,\t%f,\t%f\n", accel_pitch, accel_roll, total_acceleration_filtered,sqrt(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z));
		//printf("%d,\t%d,\t%d,\t%f,\t%f,\t%f\n", accel_raw_x, accel_raw_y, accel_raw_z, accel_x, accel_y, accel_z);
		//printf("%f,\t%f,\t%f,\t%f,\t%f,\t%f,\t%d,\t%d,\t%d,\t%f\n", gyro_rate_x, gyro_rate_y, gyro_rate_z, gyro_angle_x, gyro_angle_y, gyro_angle_z, accel_raw_x, accel_raw_y, accel_raw_z, current_time - last_time);
		//printf("%d, %d, %d\n", gyro_raw_x - gyro_cal_x, gyro_raw_y - gyro_cal_y, gyro_raw_z - gyro_cal_z);
	}
	//printf("%d\n", accel_raw_x);
}


