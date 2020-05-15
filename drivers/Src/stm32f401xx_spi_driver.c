/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: May 11, 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_gpio_driver.h"

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
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function verifies the state of any flag register.
 *
 * @param[in]         - Base address of the SPI peripheral;
 * @param[in]         - Flag name in SR register.
 * @param[in]         -
 *
 * @return            -	Flag status (SET or RESET)
 *
 * @Note              - none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}

	return FLAG_RESET;
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

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data through the Tx buffer.
 *
 * @param[in]         - Base address of the SPI peripheral;
 * @param[in]         - Tx buffer (pointer);
 * @param[in]         - Lenght of Tx buffer.
 *
 * @return            - none
 *
 * @Note              - This is actually a blocking API call;
 * 					  -	The function will wait until all the bytes are transmitted;
 * 					  - The Tx buffer is only acessible through the Data Register (DR).
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){

	while(Len > 0){
		//1. Wait until TXE is set/empty
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){
			//16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);	// Dereferencing to load the data and typecasting uint8_t to uint16_t
			Len--;	// Decrement 1 byte
			Len--;	// Decrement 2 bytes
			(uint16_t*)pTxBuffer++;	// Increment the adress pointer to the next data
		} else{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;			// Decrement 1 byte
			pTxBuffer++;	// Increment the adress pointer to the next data
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - This function enables the SPI peripheral to work properly after register configuration.
 *
 * @param[in]         - Base address of the SPI peripheral;
 * @param[in]         - Enable or Disable macro.
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - Master mode fault occurs when the master device has its NSS pin pulled to low (in NSS hardware mode)
 * 						or SSI bit low (in NSS software mode), this automatically sets the MODF bit;
 * 					  - When the SSM is enabled, for master, the NSS signal should be tied to +VCC when not used to avoid
 * 					  	MODF error which happens in multi master communication;
 * 					  - This function make SSI=1 to tie NSS to +VCC internally. SSI bit influences NSS state when SSM=1;
 * 					  - By default SSI=0, so NSS will be pulled to low which is not acceptable for master when working in
 * 					    non multi master situation.
 *
 * @param[in]         - Base address of the SPI peripheral;
 * @param[in]         - Enable or Disable macro.
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	} else{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	} else{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}