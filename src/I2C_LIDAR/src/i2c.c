#include <stdint.h>
#include "stm32f4xx.h"
#include <string.h> // for memset() declaration
#include <math.h>   // for MA_PI

#include "i2c.h"

void nano_wait(unsigned int);

#define PLL_M 4
#define PLL_N 180
#define PLL_P 0


void config (void) {
	// Enable HSE
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY));

    // Set the Power Enable clock
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    // Configure the FLASH->ACR
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

    // Configure the PRESCALARS hCLK, PCLK1, PCLK2;
    // AHB PR
    RCC -> CFGR |= RCC_CFGR_HPRE_DIV1;

    // APB1 PR
    RCC -> CFGR |= RCC_CFGR_PPRE1_DIV4;

    // APB2 PR
    RCC -> CFGR |= RCC_CFGR_PPRE2_DIV2;

    // Configure the MAIN PLL
    RCC->PLLCFGR = (PLL_M <<0) | (PLL_N << 6) | (RCC_PLLCFGR_PLLSRC_HSE);

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;

    // select the clock source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);


}

void I2Cinit (void) {
    //init i2c
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    //set pins to alternative function
    GPIOB->MODER |= (1<<17) | (1<<19);
    GPIOB->AFR[1] |= (0x04) | (0x04<<4);
    //set otyper to open drain
    GPIOB->OTYPER |= (1<<8) | (1<<9);
    GPIOB->OSPEEDR |= (0xF<<16);
    GPIOB->PUPDR |= (0x5<<16);

    I2C1->CR1 = I2C_CR1_SWRST;//reset i2c
    I2C1->CR1 &= ~I2C_CR1_SWRST;// release reset i2c
    I2C1->CR2 |= 45;

    // Set pc5 to low for Xshut
    /*RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIOC -> MODER |= 0x00000000;*/
    I2C1 -> OAR1 &= ~0x00000001;
    I2C1 -> OAR2 &= ~0x00000001;
    // Set
    I2C1->CCR = 193;
    I2C1->TRISE = 46;
    I2C1->CR1 |= I2C_CR1_PE;
}

void I2Cstart (void) {
    I2C1->CR1 |= I2C_CR1_START;
	I2C1->CR1 |= I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_SB));
}

void I2Cwrite (uint8_t data) {
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2Caddress (uint8_t address) {
    I2C1->DR = address<<1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    uint8_t temp = I2C1->SR1 | I2C1->SR2;
}

void I2Cstop (void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

void I2Cwrite2bytes (uint16_t data) {
    uint8_t datalow = data & 0x00FF;
    uint8_t datahigh = (data & 0xFF00) >> 8;

    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = datalow;
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = datahigh;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void I2CbeginTransmission(uint8_t address) {
    I2C1->DR = address<<1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    uint8_t temp = I2C1->SR1 | I2C1->SR2;
}

void I2CrequestFrom(uint8_t address,uint8_t *buffer, uint8_t numOfBytes) {
    uint8_t remaining = numOfBytes;
	if (numOfBytes == 1) {
		I2C1->DR = (address << 1)|0b00000001; // Send the Address
    	while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait for ADDR bit to set

    	I2C1->CR1 &= ~I2C_CR1_ACK;
    	uint8_t temp = I2C1->SR1 | I2C1->SR2; // READ SR1 AND SR2
    	I2C1->CR1 |= I2C_CR1_STOP;

    	buffer[numOfBytes - remaining] = I2C1->DR;
	}
	else {
		I2C1->DR = (address << 1)|0b00000001; // Send the Address
    	while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait for ADDR bit to set
    	uint8_t temp = I2C1->SR1 | I2C1->SR2;

    	while(remaining > 2) {
    		while(!(I2C1->SR1 & I2C_SR1_RXNE)); // wait for RxNE to set

    		buffer[numOfBytes - remaining] = I2C1->DR;

    		I2C1->CR1 |= I2C_CR1_ACK;

    		remaining = remaining - 1;
    	}

    	while(!(I2C1->SR1 & I2C_SR1_RXNE));
    	buffer[numOfBytes -remaining] = I2C1->DR;

    	I2C1->CR1 &= ~I2C_CR1_ACK;

    	I2C1->CR1 |= I2C_CR1_STOP;

    	remaining = remaining - 1;

    	while(!(I2C1->SR1 & I2C_SR1_RXNE));
    	buffer[numOfBytes - remaining] = I2C1->DR;
	}
    //while (!(I2C1->SR1 & I2C_SR1_BTF));
}


void init_I2C(void)
{

    
    config();
    I2Cinit();


    
    while (1) {
        
        
        uint8_t data = 0b10101010;
    	I2Cstart();
    	I2Caddress (0x29);
    	I2Cwrite(data);
    	I2Cstop();
    }
}
