/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: May 11, 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral.
 *
 * @param[in]         - Base address of the gpio peripheral;
 * @param[in]         - ENABLE or DISABLE macros.
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		} else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		} else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		} else if(pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	} else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		} else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		} else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		} else if(pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes the SPI peripheral.
 *
 * @param[in]         - It holds the base address of the SPI and SPI pin configuration settings (Handle structure).
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -	none
 *
 * @Note              - none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){

	//first lets configure the SPI_CR1 register

	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidi mode should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);

	} else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//7 . configure the SSM
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	// Writing to register CR1
	pSPIHandle->pSPIx->CR1 = tempreg;

}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function deinitializes the SPI peripheral.
 *
 * @param[in]         - Base address of the SPI peripheral.
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -	none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

	if(pSPIx == SPI1){
		SPI1_PCLK_DI();
	} else if(pSPIx == SPI2){
		SPI2_PCLK_DI();
	} else if(pSPIx == SPI3){
		SPI3_PCLK_DI();
	} else if(pSPIx == SPI4){
		SPI4_PCLK_DI();
	}
}
