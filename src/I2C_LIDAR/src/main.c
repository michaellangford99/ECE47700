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
#include <string.h> // for memset() declaration
#include <math.h>   // for MA_PI
#include "i2c.h"
//#include "VL53L1X.h"
// Be sure to change this to your login...
const char login[] = "garciads";

void nano_wait(unsigned int);
const char font[];


/*
  **
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  **
*/

#define PLL_M 4
#define PLL_N 180
#define PLL_P 0

#include "stm32f4xx.h"
#include "i2c.h"
#include "VL53L1X.h"

int main(void)
{


    //init I2C
    //init device descriptor
    //call init
    I2Cinit();
    VL53L1X_t device_descriptor;
    VL53L1X_init(&device_descriptor);
    //setAddress(0x2A + i);

    VL53L1X_startContinuous(50);
    uint16_t hold;
    while(1)
    {
    	hold = VL53L1X_read();
        printf("data!: %d\n", hold);
    }


	/*uint16_t registerToRead = 0x00E7;
	uint8_t address = 0x29;
    uint8_t data0 = 0xff & (registerToRead >> 8);
    uint8_t data1 = 0xff & (registerToRead);
    I2Cinit();
    uint8_t buffer[17];
    while (1) {
    	I2CstartWrite();
    	I2Caddress (address);
    	//I2Cwrite2bytes(registerToRead);
    	I2Cwrite(data0);
    	I2Cwrite(data1);
    	I2Cstop();
    	I2CstartRead();
    	I2CrequestFrom(address, buffer,(uint8_t)17);
    	I2Cstop();
    }*/
}
