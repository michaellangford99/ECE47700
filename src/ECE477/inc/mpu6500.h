#ifndef __MPU6500_H__
#define __MPU6500_H__

#include <stdint.h>
#include "imu.h"

//register definitions
#define MPU6500_SELF_TEST_X_GYRO 0x00
#define MPU6500_SELF_TEST_Y_GYRO 0x01
#define MPU6500_SELF_TEST_Z_GYRO 0x02
#define MPU6500_SELF_TEST_X_ACCEL 0x0D
#define MPU6500_SELF_TEST_Y_ACCEL 0x0E
#define MPU6500_SELF_TEST_Z_ACCEL 0x0F
#define MPU6500_XG_OFFSET_H 0x13
#define MPU6500_XG_OFFSET_L 0x14
#define MPU6500_YG_OFFSET_H 0x15
#define MPU6500_YG_OFFSET_L 0x16
#define MPU6500_ZG_OFFSET_H 0x17
#define MPU6500_ZG_OFFSET_L 0x18
#define MPU6500_SMPLRT_DIV 0x19
#define MPU6500_CONFIG 0x1A
#define MPU6500_GYRO_CONFIG 0x1B
#define MPU6500_ACCEL_CONFIG 0x1C
#define MPU6500_ACCEL_CONFIG_2 0x1D
#define MPU6500_LP_ACCEL_ODR 0x1E
#define MPU6500_WOM_THR 0x1F
#define MPU6500_FIFO_EN 0x23
#define MPU6500_I2C_MST_CTRL 0x24
#define MPU6500_I2C_SLV0_ADDR 0x25
#define MPU6500_I2C_SLV0_REG 0x26
#define MPU6500_I2C_SLV0_CTRL 0x27
#define MPU6500_I2C_SLV1_ADDR 0x28
#define MPU6500_I2C_SLV1_REG 0x29
#define MPU6500_I2C_SLV1_CTRL 0x2A
#define MPU6500_I2C_SLV2_ADDR 0x2B
#define MPU6500_I2C_SLV2_REG 0x2C
#define MPU6500_I2C_SLV2_CTRL 0x2D
#define MPU6500_I2C_SLV3_ADDR 0x2E
#define MPU6500_I2C_SLV3_REG 0x2F
#define MPU6500_I2C_SLV3_CTRL 0x30
#define MPU6500_I2C_SLV4_ADDR 0x31
#define MPU6500_I2C_SLV4_REG 0x32
#define MPU6500_I2C_SLV4_DO 0x33
#define MPU6500_I2C_SLV4_CTRL 0x34
#define MPU6500_I2C_SLV4_DI 0x35
#define MPU6500_I2C_MST_STATUS 0x36
#define MPU6500_INT_PIN_CFG 0x37
#define MPU6500_INT_ENABLE 0x38
#define MPU6500_INT_STATUS 0x3A
#define MPU6500_ACCEL_XOUT_H 0x3B
#define MPU6500_ACCEL_XOUT_L 0x3C
#define MPU6500_ACCEL_YOUT_H 0x3D
#define MPU6500_ACCEL_YOUT_L 0x3E
#define MPU6500_ACCEL_ZOUT_H 0x3F
#define MPU6500_ACCEL_ZOUT_L 0x40
#define MPU6500_TEMP_OUT_H 0x41
#define MPU6500_TEMP_OUT_L 0x42
#define MPU6500_GYRO_XOUT_H 0x43
#define MPU6500_GYRO_XOUT_L 0x44
#define MPU6500_GYRO_YOUT_H 0x45
#define MPU6500_GYRO_YOUT_L 0x46
#define MPU6500_GYRO_ZOUT_H 0x47
#define MPU6500_GYRO_ZOUT_L 0x48
#define MPU6500_EXT_SENS_DATA_00 0x49
#define MPU6500_EXT_SENS_DATA_01 0x4A
#define MPU6500_EXT_SENS_DATA_02 0x4B
#define MPU6500_EXT_SENS_DATA_03 0x4C
#define MPU6500_EXT_SENS_DATA_04 0x4D
#define MPU6500_EXT_SENS_DATA_05 0x4E
#define MPU6500_EXT_SENS_DATA_06 0x4F
#define MPU6500_EXT_SENS_DATA_07 0x50
#define MPU6500_EXT_SENS_DATA_08 0x51
#define MPU6500_EXT_SENS_DATA_09 0x52
#define MPU6500_EXT_SENS_DATA_10 0x53
#define MPU6500_EXT_SENS_DATA_11 0x54
#define MPU6500_EXT_SENS_DATA_12 0x55
#define MPU6500_EXT_SENS_DATA_13 0x56
#define MPU6500_EXT_SENS_DATA_14 0x57
#define MPU6500_EXT_SENS_DATA_15 0x58
#define MPU6500_EXT_SENS_DATA_16 0x59
#define MPU6500_EXT_SENS_DATA_17 0x5A
#define MPU6500_EXT_SENS_DATA_18 0x5B
#define MPU6500_EXT_SENS_DATA_19 0x5C
#define MPU6500_EXT_SENS_DATA_20 0x5D
#define MPU6500_EXT_SENS_DATA_21 0x5E
#define MPU6500_EXT_SENS_DATA_22 0x5F
#define MPU6500_EXT_SENS_DATA_23 0x60
#define MPU6500_I2C_SLV0_DO 0x63
#define MPU6500_I2C_SLV1_DO 0x64
#define MPU6500_I2C_SLV2_DO 0x65
#define MPU6500_I2C_SLV3_DO 0x66
#define MPU6500_I2C_MST_DELAY_CTRL 0x67
#define MPU6500_SIGNAL_PATH_RESET 0x68
#define MPU6500_MOT_DETECT_CTRL 0x69
#define MPU6500_USER_CTRL 0x6A
#define MPU6500_PWR_MGMT_1 0x6B
#define MPU6500_PWR_MGMT_2 0x6C
#define MPU6500_FIFO_COUNTH 0x72
#define MPU6500_FIFO_COUNTL 0x73
#define MPU6500_FIFO_R_W 0x74
#define MPU6500_WHO_AM_I 0x75
#define MPU6500_XA_OFFSET_H 0x77
#define MPU6500_XA_OFFSET_L 0x78
#define MPU6500_YA_OFFSET_H 0x7A
#define MPU6500_YA_OFFSET_L 0x7B
#define MPU6500_ZA_OFFSET_H 0x7D
#define MPU6500_ZA_OFFSET_L 0x7E

typedef union {
	struct {
		int XGYRO_Cten : 1;//X Gyro self-test
		int YGYRO_Cten : 1;//Y Gyro self-test
		int ZGYRO_Cten : 1;//Z Gyro self-test
		
		int GYRO_FS_SEL : 2;//Gryo full scale select

		int reserved : 1;//Reserved
		int Fchoice_b : 2;//Used to bypass DLPF

	} bits;
	uint8_t raw;
} Gyro_Config;

typedef enum {
	_250DPS = 0,
	_500DPS,
	_1000DPS,
	_2000DPS
} Gyro_FS_Sel;


typedef union {
	struct{
		int ax_st_en : 1;//X Accel self-test
		int ay_st_en : 1;//Y Accel self-test
		int az_st_en : 1;//Z Accel self-test

		int ACCEL_FS_SEL : 2;//Accel Full Scale Select
		int res : 3;//Reserved
	} bits;
	uint8_t raw;
} Accel_Config;

typedef enum {
	_2g = 0,
	_4g,
	_8g,
	_16g
} Accel_FS_Sel;

/***********************************************
 * G_GAIN TABLE VALUES
 * +-125	4.375/1000
 * +-250	8.75/1000
 * +-500	17.5/1000
 * +-1000	35/1000
 * +-2000	70/1000
 *
 * A_GAIN TABLE VALUES
 * +-2		0.061/1000
 * +-4		0.122/1000
 * +-8		0.244/1000
 * +-16		0.488/1000
 *
 ***********************************************/

#define G_GAIN_125DPS   0.004375f
#define G_GAIN_250DPS   (250.0f/32768.0f)
#define G_GAIN_500DPS   (500.05/32768.0f)
#define G_GAIN_1000DPS  0.035f
#define G_GAIN_2000DPS  0.070f

#define A_GAIN_2G       0.000061f
#define A_GAIN_4G       0.000122f
#define A_GAIN_8G       0.000244f
#define A_GAIN_16G      0.000488f

#define A_RAW_1G        (32768/2)

#define G_GAIN 1//G_GAIN_250DPS //gyroscope gain to convert to degrees per second
#define A_GAIN 1//A_GAIN_2G //accelerometer gain to convert to g's

//Calibration array length
#define CAL_LENGTH 4096

extern imu_data_t mpu6500_data;

void init_MPU6500(void);
void calibrate_MPU6500();
void read_axes_MPU6500();
void update_MPU6500(void);

#endif /* __MPU6500_H__ */
