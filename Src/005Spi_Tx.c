/*
 * 005Spi_Tx.c
 *
 *  Created on: 13 de mai de 2020
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

	/* For this application we don't want to use MISO and NSS because there is no slave */
	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	//GPIO_Init(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	//GPIO_Init(&SPIPins);
}

void SPI1_Inits(void){

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; //generates sclk of 8MHz
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // Software slave management enabled enabled for NSS pin

	SPI_Init(&SPI1handle);
}

int main(void){

	char user_data[] = "Hello world";

	/* Enabling SPI1 peripheral */
	SPI_PeriClockControl(SPI1, ENABLE);

	// This function is used to initialize the GPIO pins to behave as SPI1 pins
	SPI1_GPIOInits();

	// This function is used to initialize the SPI1 peripheral parameters
	SPI1_Inits();

	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI1, ENABLE);

	while(1){

		// Enable the SPI1 peripheral after have done all register configurations
		SPI_PeripheralControl(SPI1, ENABLE);

		// Send data
		SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

		//lets confirm SPI is not busy
		while(SPI_GetFlagStatus(SPI1, SPI_BUSY_FLAG));

		// Disable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, DISABLE);
	}

}

