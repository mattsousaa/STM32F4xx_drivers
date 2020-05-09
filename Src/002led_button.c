/*
 * 002led_button.c
 *
 *  Created on: 7 de mai de 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

void delay(void){
	for(uint32_t i = 0; i < 50000/2; i++);
}

int main(void){

	GPIO_Handle_t GpioLed, GpioBut;

	/* LED config */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* BUTTON config */
	GpioBut.pGPIOx = GPIOB;
	GpioBut.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBut.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBut.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBut.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBut);

	while(1){
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == LOW){
			//delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_14);
		}
	}

	return 0;
}

