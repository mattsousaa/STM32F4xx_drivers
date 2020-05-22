################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/009I2C_master_tx_testing.c 

OBJS += \
./Src/009I2C_master_tx_testing.o 

C_DEPS += \
./Src/009I2C_master_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/009I2C_master_tx_testing.o: ../Src/009I2C_master_tx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F401RETx -DSTM32F4 -DDEBUG -c -I../Inc -I"C:/Users/mateu/Documents/Curso STM32/Projetos - STM32/STM32F4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/009I2C_master_tx_testing.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

