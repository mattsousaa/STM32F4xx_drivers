/*
 * 003HSI_Measurement.c
 *
 *  Created on: 9 de mai de 2020
 *      Author: Mateus Sousa
 */


#include "stm32f401xx.h"
#include "stm32f401xx_gpio_driver.h"

int main(void){

	GPIO_Handle_t GpioLed;
	RCC_RegDef_t *RCC_RegDef = (RCC_RegDef_t*) RCC_BASEADDR;

	//Configure the RCC_CFGR MCO1 bit fields to select HSI as clock source
	RCC_RegDef->CFGR &= ~(0x3 << 21); //clear 21 and 22 bit positions

	//Configure MCO1 prescaler
	RCC_RegDef->CFGR |= (0x7 << 24);

	/* LED config */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;	//Configure the mode of GPIOA pin 8 as alternate function mode

	GPIO_PeriClockControl(GPIOA, ENABLE);	//Enable the peripheral clock for GPIOA peripheral

	GPIO_Init(&GpioLed);

	while(1);
}
