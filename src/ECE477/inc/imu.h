#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include "system.h"
#include "string.h"

typedef struct {
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

	float accel_pitch;
	float accel_roll;

	float compl_pitch;
	float compl_roll;
} imu_data_t;

#endif /* __IMU_H__ */
