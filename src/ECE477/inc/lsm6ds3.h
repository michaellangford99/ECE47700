#ifndef __LSM6DS3_H__
#define __LSM6DS3_H__

//LSM6DSO definitions for register addresses and pins in the register
#define FUNC_CFG_ACCESS 0x01
#define WHO_AM_I 0x0f //read only
#define FIFO_CTRL1 0x07
#define FIFO_CTRL2 0x08
#define FIFO_CTRL3 0x09
#define FIFO_CTRL4 0x0a
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define CTRL3_G 0x12
#define CTRL4_G 0x13
#define CTRL5_G 0x14
#define CTRL6_G 0x15
#define CTRL7_G 0x16
#define CTRL8_XL 0x17
#define CTRL9_XL 0x18
#define CTRL10_C 0x19
#define OUT_TEMP_L 0x20 //read only
#define OUT_TEMP_H 0x21 //read only
#define OUTX_L_G 0x22 //read only
#define OUTX_H_G 0x23 //read only
#define OUTY_L_G 0x24 //read only
#define OUTY_H_G 0x25 //read only
#define OUTZ_L_G 0x26 //read only
#define OUTZ_H_G 0x27 //read only
#define OUTX_L_A 0x28 //read only
#define OUTX_H_A 0x29 //read only
#define OUTY_L_A 0x2a //read only
#define OUTY_H_A 0x2b //read only
#define OUTZ_L_A 0x2c //read only
#define OUTZ_H_A 0x2d //read only

//Register definitions for control registers
/***********************************************
 * CTRL1_XL Definitions (Refer to Table 52 for Accel ODR Settings)
 * FS SELECTION
 * +-2	00
 * +-4	10
 * +-8 	11
 * +-16 01
 ************************************************/
#define ODR_XL3 	(1<<7)
#define ODR_XL2 	(1<<6)
#define ODR_XL1 	(1<<5)
#define ODR_XL0 	(1<<4)
#define FS_XL1 		(1<<3)
#define FS_XL0 		(1<<2)
#define LPF1_BW_SEL (1<<1)
#define BW0_XL		(1<<0)

#define CTRL1_DEF 0b00000000

#define ACCEL_FS_2G  (0)
#define ACCEL_FS_4G  (FS_XL1 |      0)
#define ACCEL_FS_8G  (FS_XL1 | FS_XL0)
#define ACCEL_FS_18G (0      | FS_XL0)

/***********************************************
 * CTRL2_G Definitions (See Table 55 for Gyro ODR Settings)
 * FS SELECTION
 * +-250	00
 * +-500	01
 * +-1000	10
 * +-2000	11
 ************************************************/
#define ODR_G3 		(1<<7)
#define ODR_G2 		(1<<6)
#define ODR_G1 		(1<<5)
#define ODR_G0 		(1<<4)
#define FS_G1 		(1<<3)
#define FS_G0 		(1<<2)
#define FS_125 		(1<<1)

#define CTRL2_DEF 0b00000000

#define GYRO_FS_250DPS  (0)
#define GYRO_FS_500DPS  (FS_XL1 |      0)
#define GYRO_FS_1000DPS (FS_XL1 | FS_XL0)
#define GYRO_FS_2000DPS (0      | FS_XL0)

/***********************************************
 * CTRL3_C Definitions
 ************************************************/
#define BOOT 		(1<<7)
#define BDU 		(1<<6)
#define H_LACTIVE	(1<<5)
#define PP_OD		(1<<4)
#define SIM 		(1<<3)
#define IF_INC 		(1<<2)
#define BLE 		(1<<1)
#define SW_RESET	(1<<0)

#define CTRL3_DEF 0b00000000

/***********************************************
 * CTRL4_C Definitions
 ************************************************/
#define DEN_XL_EN 		(1<<7)
#define SLEEP 			(1<<6)
#define INT2_on_INT1	(1<<5)
#define DEN_DRDY_INT1 	(1<<4)
#define DRDY_MASK 		(1<<3)
#define I2C_disable 	(1<<2)
#define LPF1_SEL_G 		(1<<1)

#define CTRL4_DEF 0b00000000

/***********************************************
 * CTRL5_C Definitions (Refer to Tables 62-64)
 ************************************************/
#define ROUNDING2 	(1<<7)
#define ROUNDING1	(1<<6)
#define ROUNDING0 	(1<<5)
#define DEN_LH 		(1<<4)
#define ST1_G 		(1<<3)
#define ST0_G 		(1<<2)
#define ST1_XL 		(1<<1)
#define ST0_XL		(1<<0)

#define CTRL5_DEF 0b00000000

/***********************************************
 * CTRL6_C Definitions (FTYPE refer to Table 68)
 ************************************************/
#define TRIG_EN 	(1<<7)
#define LVL_EN 		(1<<6)
#define LVL2_EN 	(1<<5)
#define XL_HM_MODE 	(1<<4)
#define USR_OFF_W 	(1<<3)
#define FTYPE_1 	(1<<1)
#define FTYPE_0		(1<<0)

#define CTRL6_DEF 0b00000000

/***********************************************
 * CTRL7_G Definitions
 ************************************************/
#define G_HM_MODE 		(1<<7)
#define HP_EN_G 		(1<<6)
#define HPM1_G 			(1<<5)
#define HPM0_G 			(1<<4)
#define ROUNDING_STATUS	(1<<2)

#define CTRL7_DEF 0b00000000

/***********************************************
 * CTRL8_XL Definitions
 ************************************************/
#define LPF2_XL_EN 		(1<<7)
#define HPCF_XL1 		(1<<6)
#define HPCF_XL0 		(1<<5)
#define HP_REF_MODE 	(1<<4)
#define INPUT_COMPOSITE	(1<<3)
#define HP_SLOPE_XL_EN 	(1<<2)
#define LOW_PASS_ON_6D	(1<<0)

#define CTRL8_DEF 0b00000000

/***********************************************
 * CTRL9_XL Definitions
 ************************************************/
#define DEN_X 		(1<<7)
#define DEN_Y 		(1<<6)
#define DEN_Z 		(1<<5)
#define DEN_XL_G 	(1<<4)
#define SOFT_EN 	(1<<2)

#define CTRL9_DEF 0b11100000

/***********************************************
 * CTRL10C Definitions
 ************************************************/
#define WRIST_TILT_EN 	(1<<7)
#define TIMER_EN 		(1<<5)
#define PEDO_EN 		(1<<4)
#define TILT_EN 		(1<<3)
#define FUNC_EN 		(1<<2)
#define PEDI_RST_STEP 	(1<<1)
#define SIGN_MOTION_EN	(1<<0)

#define CTRL10_DEF 0b00000000

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

#define G_GAIN 1//G_GAIN_250DPS //gyroscope gain to convert to degrees per second
#define A_GAIN 1//A_GAIN_2G //accelerometer gain to convert to g's

//Calibration array length
#define CAL_LENGTH 4096

void init_LSM6DS3(void);
void calibrate_LSM6DS3();
void read_axes(void);
void update_LSM6DS3(void);

#endif /* __LSM6DS3_H__ */
