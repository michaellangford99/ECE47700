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

int16_t data[6] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

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
	//register addresses
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

	//inital LSM6DSO parameters (see LSM6DSO reference manual (https://www.st.com/en/mems-and-sensors/lsm6dsl.html))
	uint8_t ctrl1 = CTRL1_DEF | ODR_XL2 | ODR_XL1;
	uint8_t ctrl2 = CTRL2_DEF | ODR_G3  | ODR_G1;
	uint8_t ctrl3 = CTRL3_DEF;
	uint8_t ctrl4 = CTRL4_DEF | I2C_disable;
	uint8_t ctrl5 = CTRL5_DEF;
	uint8_t ctrl6 = CTRL6_DEF;
	uint8_t ctrl7 = CTRL7_DEF;
	uint8_t ctrl8 = CTRL8_DEF;
	uint8_t ctrl9 = CTRL9_DEF;
	uint8_t ctrl10 = CTRL10_DEF;

	//ctrl register writing
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

void read_axes(){
	uint8_t data_buf[12];
	for (int i = 0; i < 12; i++)
	{
		data_buf[i] = readReg(OUTX_L_G+i);
		data_buf[i] = readReg(OUTX_L_G+i);
	}

	data[0] = ((int16_t)data_buf[0] | ((int16_t)data_buf[1] << 8)) * G_GAIN;
	data[1] = ((int16_t)data_buf[2] | ((int16_t)data_buf[3] << 8)) * G_GAIN;
	data[2] = ((int16_t)data_buf[4] | ((int16_t)data_buf[5] << 8)) * G_GAIN;
	data[3] = ((int16_t)data_buf[6] | ((int16_t)data_buf[7] << 8)) * A_GAIN;
	data[4] = ((int16_t)data_buf[8] | ((int16_t)data_buf[9] << 8)) * A_GAIN;
	data[5] = ((int16_t)data_buf[10] | ((int16_t)data_buf[11] << 8)) * A_GAIN;
}

void test_LSM6DS3(void){
    //init_spi();
    //initLSM();
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

    for (int i = 0; i < CAL_LENGTH; i++)
    {
    	for(int i = 0; i < 1000; i++){
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
    int32_t xyz[3];

    xyz[0] = 0;
    xyz[1] = 0;
    xyz[2] = 0;

    while(1){

    	uint8_t data_buf[12];
    	for (int i = 0; i < 12; i++)
    	{
    		data_buf[i] = readReg(xgyroL+i);
    		data_buf[i] = readReg(xgyroL+i);
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

		xyz[0] += data[0];
		xyz[1] += data[1];
		xyz[2] += data[2];

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

		if (p++ > 40)
		{
			printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", data[0], data[1], data[2], data[3], data[4], data[5], xyz[0], xyz[1], xyz[2]);
			p=0;
		}
    }
}
