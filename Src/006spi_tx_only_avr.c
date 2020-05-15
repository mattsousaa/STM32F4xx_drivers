/*
 * 006spi_tx_only_avr.c
 *
 *  Created on: 14 de mai de 2020
 *      Author: Mateus Sousa
 */

#include <string.h>
#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_spi_driver.h"

/*
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 --> SPI1_SCLK
 * PA4 --> SPI1_NSS
 * ALT function mode: 5
 * */

void delay(void){
	for(uint32_t i = 0; i < 400000/2; i++);
}

void SPI1_GPIOInits(void){

	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	/* Enabling GPIOA peripheral */
	GPIO_PeriClockControl(GPIOA, ENABLE);

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);

	/* For this application we don't want to use MISO because the master don't receives data */
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}

void SPI1_Inits(void){

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //generates sclk of 2MHz
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enabled enabled for NSS pin

	SPI_Init(&SPI1handle);
}

int main(void){

	char user_data[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers";
	GPIO_Handle_t GpioLed, GpioBut;

	/* LED config */
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	/* BUTTON config */
	GpioBut.pGPIOx = GPIOB;
	GpioBut.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBut.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBut.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioBut.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBut);

	/* Enabling SPI1 peripheral */
	SPI_PeriClockControl(SPI1, ENABLE);

	// This function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	// This function is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();

	/*
	* Making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI1, ENABLE);

	while(1){

		//wait till button is pressed
		while(GPIO_ReadFromInputPin(GPIOB,GPIO_PIN_NO_12));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// Enable the SPI1 peripheral after have done all register configurations
		SPI_PeripheralControl(SPI1, ENABLE);

		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_14);

		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI1, &dataLen, 1);

		// Send data
		SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

		// Disable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, DISABLE);
	}
}


