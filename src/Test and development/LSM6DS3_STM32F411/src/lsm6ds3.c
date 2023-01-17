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
#include "string.h"
#include "spi.h"
#include "lsm6ds3.h"
#include "systick.h"

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
#define OUTZ_H_A 0x2d //read only'

//Register definitions for control registers

//first off, we want to figure out if we are doing fixed point or what
//2 g max means +/- 32768 covers the range +/- 2g
//1g = 9.81 m/s^2
//so a raw  number->
//converted_to_gs = (raw / 32768.0f)*(2.0f)*(9.81f)

//degrees per second
//first of all, systick needs to have high precision
//not sure what the range on this is.


#define G_GAIN 1//0.00875; //gyroscope gain to convert to degrees per second
#define A_GAIN 1//0.00061; //accelerometer gain to convert to g's??

int16_t data[6] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

void init_LSM6DS3(){
	//register addresses (see define statements)
	//uint8_t addr0 = FUNC_CFG_ACCESS;
	uint8_t addr1 = CTRL1_XL;
	uint8_t addr2 = CTRL2_G;
	uint8_t addr3 = CTRL3_G;
	uint8_t addr4 = CTRL4_G;
	uint8_t addr5 = CTRL5_G;
	uint8_t addr6 = CTRL6_G;
	uint8_t addr7 = CTRL7_G;
	uint8_t addr8 = CTRL8_XL;
	uint8_t addr9 = CTRL9_XL;
	uint8_t addr10 = CTRL10_C;

	//inital LSM6DSO parameters (see LSM6DSO reference manual (https://www.st.com/en/mems-and-sensors/lsm6dsr.html))
	//uint8_t ctrl0 = 0b10000000;
	uint8_t ctrl1 = 0b01100000;
	uint8_t ctrl2 = 0b01100000;
	uint8_t ctrl3 = 0b00000000;
	uint8_t ctrl4 = 0b00000100;
	uint8_t ctrl5 = 0b00000000;
	uint8_t ctrl6 = 0b00000000;
	uint8_t ctrl7 = 0b00000000;
	uint8_t ctrl8 = 0b00000000;
	uint8_t ctrl9 = 0b11100010;
	uint8_t ctrl10 = 0b00000000;

	//ctrl register writing
	//writeReg(addr0, ctrl0);
	writeReg(addr1, ctrl1);
	writeReg(addr2, ctrl2);
	writeReg(addr3, ctrl3);
	writeReg(addr4, ctrl4);
	writeReg(addr5, ctrl5);
	writeReg(addr6, ctrl6);
	writeReg(addr7, ctrl7);
	writeReg(addr8, ctrl8);
	writeReg(addr9, ctrl9);
	writeReg(addr10, ctrl10);
}

void read_axes()
{
	uint8_t data_buf[12];
	for (int i = 0; i < 12; i++)
	{
		data_buf[i] = readReg(OUTX_L_G+i);
		data_buf[i] = readReg(OUTX_L_G+i);
		//data_buf[i] = readReg(xgyroL+i);
	}

	data[0] = ((int16_t)data_buf[0] | ((int16_t)data_buf[1] << 8)) * G_GAIN;
	data[1] = ((int16_t)data_buf[2] | ((int16_t)data_buf[3] << 8)) * G_GAIN;
	data[2] = ((int16_t)data_buf[4] | ((int16_t)data_buf[5] << 8)) * G_GAIN;
	data[3] = ((int16_t)data_buf[6] | ((int16_t)data_buf[7] << 8)) * A_GAIN;
	data[4] = ((int16_t)data_buf[8] | ((int16_t)data_buf[9] << 8)) * A_GAIN;
	data[5] = ((int16_t)data_buf[10] | ((int16_t)data_buf[11] << 8)) * A_GAIN;
}

void test_LSM6DS3(void)
{
    //init_spi1();
    //initLSM();
    //init_UART();
    /*while(1){
    	uint8_t whoami = OUTX_L_G;
    	uint8_t read;
    	read = readReg(whoami);
        printf("Hello");
    	for(int i = 0; i < 10000; i++){
    	    asm("NOP");
    	}
    }*/
    uint8_t xgyroL = OUTX_L_G;
    uint8_t xgyroH = OUTX_H_G;
    uint8_t ygyroL = OUTY_L_G;
    uint8_t ygyroH = OUTY_H_G;
    uint8_t zgyroL = OUTZ_L_G;
    uint8_t zgyroH = OUTZ_H_G;

    uint8_t xaccelL = OUTX_L_A;
    uint8_t xaccelH = OUTX_H_A;
    uint8_t yaccelL = OUTY_L_A;
    uint8_t yaccelH = OUTY_H_A;
    uint8_t zaccelL = OUTZ_L_A;
    uint8_t zaccelH = OUTZ_H_A;

    /*while(1){
        int8_t inte = readReg(xgyroH);
        printf("%d\n", inte);
        //printf("%d\n", data[1]);
        //printf("%d\n", data[2]);
        //printf("%d\n", data[3]);
        //printf("%d\n", data[4]);
        //printf("%d\n", data[5]);

        for(int i = 0; i < 10000; i++){
            	    __asm("NOP");
        }
    }*/

    uint64_t cal[3];

    cal[0] = 0;
    cal[1] = 0;
    cal[2] = 0;

    for(int i = 0; i < 100000; i++){
        		__asm("NOP");
    		}

#define CAL_LENGTH 4096
    for (int i = 0; i < CAL_LENGTH; i++)
    {
    	for(int i = 0; i < 10000; i++){
    		__asm("NOP");
		}
    	read_axes();
    	cal[0] += data[0];
		cal[1] += data[1];
		cal[2] += data[2];
    }

    cal[0] = cal[0] / CAL_LENGTH;
    cal[1] = cal[1] / CAL_LENGTH;
    cal[2] = cal[2] / CAL_LENGTH;

    int p = 0;
    float xyz[3];

    xyz[0] = 0.0f;
    xyz[1] = 0.0f;
    xyz[2] = 0.0f;

    float last_time = 0.0f;
    float current_time = 0.0f;
    while(1){

    	uint8_t data_buf[12];
    	for (int i = 0; i < 12; i++)
    	{
    		data_buf[i] = readReg(xgyroL+i);
    		data_buf[i] = readReg(xgyroL+i);
    		//data_buf[i] = readReg(xgyroL+i);
    	}

    	data[0] = ((int16_t)data_buf[0] | ((int16_t)data_buf[1] << 8)) * G_GAIN;
		data[1] = ((int16_t)data_buf[2] | ((int16_t)data_buf[3] << 8)) * G_GAIN;
		data[2] = ((int16_t)data_buf[4] | ((int16_t)data_buf[5] << 8)) * G_GAIN;
		data[3] = ((int16_t)data_buf[6] | ((int16_t)data_buf[7] << 8)) * A_GAIN;
		data[4] = ((int16_t)data_buf[8] | ((int16_t)data_buf[9] << 8)) * A_GAIN;
		data[5] = ((int16_t)data_buf[10] | ((int16_t)data_buf[11] << 8)) * A_GAIN;

		data[0] -= cal[0];
		data[1] -= cal[1];
		data[2] -= cal[2];

		//245 DPS full scale
		//so dps = raw * 245 / (2^15-1)

		last_time = current_time;
		current_time = ftime();
		float delta_t = current_time-last_time;

		if (delta_t < 0.01f)
		{
			xyz[0] += ((float)data[0]) * delta_t * 245.0f / 32768.0f;
			xyz[1] += ((float)data[1]) * delta_t * 245.0f / 32768.0f;
			xyz[2] += ((float)data[2]) * delta_t * 245.0f / 32768.0f;
		}

        //data[0] = ((int16_t)readReg(xgyroL) | ((int16_t)readReg(xgyroH) << 8)) * G_GAIN;
        //data[0] = ((int16_t)readReg(xgyroL) | ((int16_t)readReg(xgyroH) << 8)) * G_GAIN;
        //data[1] = (/*readReg(ygyroL) | */((int16_t)readReg(ygyroH) << 8)) * G_GAIN;
        //data[1] = (/*readReg(ygyroL) | */((int16_t)readReg(ygyroH) << 8)) * G_GAIN;
        //data[2] = (/*readReg(zgyroL) | */((int16_t)readReg(zgyroH) << 8)) * G_GAIN;
        //data[2] = (/*readReg(zgyroL) | */((int16_t)readReg(zgyroH) << 8)) * G_GAIN;
        //data[3] = (/*readReg(xaccelL) | */((int16_t)readReg(xaccelH) << 8)) * A_GAIN;
        //data[3] = (/*readReg(xaccelL) | */((int16_t)readReg(xaccelH) << 8)) * A_GAIN;
        //data[4] = (/*readReg(yaccelL) | */((int16_t)readReg(yaccelH) << 8)) * A_GAIN;
        //data[4] = (/*readReg(yaccelL) | */((int16_t)readReg(yaccelH) << 8)) * A_GAIN;
        //data[5] = (/*readReg(zaccelL) |*/ ((int16_t)readReg(zaccelH) << 8)) * A_GAIN;
        //data[5] = (/*readReg(zaccelL) |*/ ((int16_t)readReg(zaccelH) << 8)) * A_GAIN;



        /*data[0] = (readReg(xgyroH) << 8);
    	data[0] |= readReg(xgyroL);
    	data[1] = (readReg(ygyroH) << 8);
    	data[1] |= readReg(ygyroL);
    	data[2] = (readReg(zgyroH) << 8);
    	data[2] |= readReg(zgyroL);
    	data[3] = (readReg(xaccelH) << 8);
    	data[3] |= readReg(xaccelL);
    	data[4] = (readReg(yaccelH) << 8);
    	data[4] |= readReg(yaccelL);
    	data[5] = (readReg(zaccelH) << 8);
    	data[5] |= readReg(zaccelL);*/

    	//printf("%d\n", data[0]);
    	//printf("%d\n", data[1]);
    	//printf("%d\n", data[2]);
    	//printf("%d\n", data[3]);
    	//printf("%d\n", data[4]);
    	//printf("%d\n", data[5]);

    	//printf("Gyro Values: X=%d Y=%d Z=%d\n", data[0], data[1], data[2]);
    	//printf("Accel Values: X=%d Y=%d Z=%d\n", data[3], data[4], data[5]);

		if (p++ > 8)
		{
			printf("%d, %d, %d, %d, %d, %d, %f, %f, %f, %f\n", data[0], data[1], data[2], data[3], data[4], data[5], xyz[0], xyz[1], xyz[2], 1.0f/delta_t);
			p=0;
		}

    	/*for(int i = 0; i < 1000; i++){
    	    __asm("NOP");
    	}*/


    }
    return 0;
}
