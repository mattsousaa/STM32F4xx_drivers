################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include drivers/Src/subdir.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
STM32F4xx_drivers.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
STM32F4xx_drivers.list \

OBJCOPY_HEX += \
STM32F4xx_drivers.hex \

OBJCOPY_BIN += \
STM32F4xx_drivers.bin \


# All Target
all: STM32F4xx_drivers.elf secondary-outputs

# Tool invocations
STM32F4xx_drivers.elf: $(OBJS) $(USER_OBJS) C:\Users\mateu\Documents\Curso\ STM32\Projetos\ -\ STM32\STM32F4xx_drivers\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "STM32F4xx_drivers.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\mateu\Documents\Curso STM32\Projetos - STM32\STM32F4xx_drivers\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="STM32F4xx_drivers.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

STM32F4xx_drivers.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "STM32F4xx_drivers.list"
	@echo 'Finished building: $@'
	@echo ' '

STM32F4xx_drivers.hex: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "STM32F4xx_drivers.hex"
	@echo 'Finished building: $@'
	@echo ' '

STM32F4xx_drivers.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "STM32F4xx_drivers.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
