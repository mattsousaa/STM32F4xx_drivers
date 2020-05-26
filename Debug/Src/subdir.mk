################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/013i2c_slave_tx_string2.c 

OBJS += \
./Src/013i2c_slave_tx_string2.o 

C_DEPS += \
./Src/013i2c_slave_tx_string2.d 


# Each subdirectory must supply rules for building sources it contributes
Src/013i2c_slave_tx_string2.o: ../Src/013i2c_slave_tx_string2.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F401RETx -DSTM32F4 -DDEBUG -c -I../Inc -I"C:/Users/mateu/Documents/Curso STM32/Projetos - STM32/STM32F4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/013i2c_slave_tx_string2.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

