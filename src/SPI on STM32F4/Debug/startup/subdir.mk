################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../startup/startup_stm32f411xe.s 

OBJS += \
./startup/startup_stm32f411xe.o 


# Each subdirectory must supply rules for building sources it contributes
startup/%.o: ../startup/%.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -I"C:/Users/Michael/OneDrive - purdue.edu/Senior_Year/Fall_2022/ECE47700/src/SPI on STM32F4/StdPeriph_Driver/inc" -I"C:/Users/Michael/OneDrive - purdue.edu/Senior_Year/Fall_2022/ECE47700/src/SPI on STM32F4/inc" -I"C:/Users/Michael/OneDrive - purdue.edu/Senior_Year/Fall_2022/ECE47700/src/SPI on STM32F4/CMSIS/device" -I"C:/Users/Michael/OneDrive - purdue.edu/Senior_Year/Fall_2022/ECE47700/src/SPI on STM32F4/CMSIS/core" -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

