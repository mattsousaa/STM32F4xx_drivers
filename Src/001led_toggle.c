/*
 * 001led_toggle.c
 *
 *  Created on: 7 de mai de 2020
 *      Author: Mateus Sousa
 */


#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

void delay(void){
	for(uint32_t i = 0; i < 50000; i++);
}

int main(void){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);

	while(1){
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
