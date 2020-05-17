/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: May 11, 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_spi_driver.h"
#include "stm32f401xx_gpio_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/**************************************************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI port
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - Macros: Enable or Disable
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 **************************************************************************************************/
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

void teste(void){

	/* LED config */

	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	//GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_14);
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_14, HIGH);
}

/*****************************************************************************************
 * @fn				- SPI_Init
 *
 * @brief			- This function initializes SPI peripherals
 *
 * @param[in]		- Pointer to SPI Handle structure
 *
 * @return			- None
 *
 * @Note			- None
 *
 *****************************************************************************************/
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

/*****************************************************************************************
 * @fn			- SPI_DeInit
 *
 * @brief		- This function de-initialize SPI peripherals
 *
 * @param[in]	- Base address of the SPI peripheral
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************************************/
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

/*****************************************************************************************
 * @fn			- SPI_GetFlagStatus
 *
 * @brief		- This function returns if bit in status register is set or not
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Name of the flag
 *
 * @return		- Flag status (True/False)
 *
 * @Note		- None
 *
 *****************************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName){

	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/*****************************************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data over SPI peripheral
 *
 * @param[in]         - Base address of the SPI peripheral
 * @param[in]         - Tx buffer
 * @param[in]         - Lenght of Tx buffer
 *
 * @return            - none
 *
 * @Note              - This is actually a blocking API call;
 * 					  -	The function will wait until all the bytes are transmitted;
 * 					  - The Tx buffer is only acessible through the Data Register (DR).
 *
 *****************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Lenght){

	while(Lenght > 0){
		/* 1. Wait until TXE is set/empty */

		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == (uint8_t)FLAG_RESET);

		/* 2. check the DFF bit in CR1 */
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){

			/* Load data into data register */
			/* 16 bit DFF */
			pSPIx->DR = *((uint16_t*)pTxBuffer);	/* Dereferencing to load the data and typecasting uint8_t to uint16_t */
			Lenght--;								/* Decrement 1 byte */
			Lenght--;								/* Decrement 2 bytes */
			(uint16_t*)pTxBuffer++;					/* Increment the adress pointer to the next data */
		} else{
			/* 8 bit DFF */
			pSPIx->DR = *(pTxBuffer);
			Lenght--;						/* Decrement 1 byte */
			pTxBuffer++;					/* Increment the adress pointer to the next data */
		}
	}
}

/*****************************************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data through the Rx buffer.
 *
 * @param[in]         - Base address of the SPI peripheral;
 * @param[in]         - Rx buffer (pointer);
 * @param[in]         - Lenght of Rx buffer.
 *
 * @return            - none
 *
 * @Note              - This is actually a blocking API call;
 * 					  -	The function will wait until all the bytes are transmitted;
 * 					  - The Rx buffer is only acessible through the Data Register (DR).
 *****************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Lenght){

	while(Lenght > 0){
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		//2. check the DFF bit in CR1
		if((pSPIx->CR1 & (1 << SPI_CR1_DFF))){

			//16 bit DFF
			//1. load the data from DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR ;
			Lenght--;
			Lenght--;
			(uint16_t*)pRxBuffer++;
		} else{
			//8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Lenght--;
			pRxBuffer++;
		}
	}
}

/*****************************************************************
 * @fn			- SPI_SendDataIT
 *
 * @brief		- This function sends data over SPI peripheral in Interrupt mode
 *
 * @param[in]	- Pointer to SPI Handle structure
 * @param[in]	- Transmit buffer
 * @param[in]	- Length of Tx buffer
 *
 * @return		- Tx State
 *
 * @Note		- None
 *
 *****************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length){

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){

		/*1. Save Tx buffer address and length information in global variables */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Length;

		/*2. Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/*3. Enable TXEIE control bit to get interrupt whenever TXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}
	/* DBG->Data transmission*/
	return state;
}

/*****************************************************************
 * @fn			- SPI_ReceiveDataIT
 *
 * @brief		- This function receives data over SPI peripheral in Interrupt mode
 *
 * @param[in]	- Pointer to SPI Handle structure
 * @param[in]	- Transmit buffer
 * @param[in]	- Length of Rx buffer
 *
 * @return		- Rx State
 *
 * @Note		- None
 *
 *****************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length){

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){

		/* Save Rx buffer address and length information in global variables */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Length;

		/* Mark SPI state as busy so that no other code can take over SPI peripheral until transmission is over */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/* Enable RXNEIE control bit to get interrupt whenever RXE flag is set in SR */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}


/*
 * IRQ Configuration and ISR handling
 */

/*****************************************************************
 * @fn			- SPI_IRQConfig
 *
 * @brief		- This function configures interrupt
 *
 * @param[in]	- IRQ Interrupt number
 * @param[in]	- Macro: Enable/Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(IRQNumber <= 31){							/* IRQ numbers(range): 0, ... , 31 */
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64){	/* IRQ numbers(range): 32, ... , 63 */
			//program ISER1 register

			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		} else if(IRQNumber >= 64 && IRQNumber < 96){	/* IRQ numbers(range): 64, ... , 95 */
			//program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	} else{

		if(IRQNumber <= 31){							/* IRQ numbers(range): 0, ... , 31 */
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if(IRQNumber > 31 && IRQNumber < 64){	/* IRQ numbers(range): 32, ... , 63 */
			//program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		} else if(IRQNumber >= 64 && IRQNumber < 96){	/* IRQ numbers(range): 64, ... , 95 */
			//program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

/*****************************************************************
 * @fn			- SPI_IRQPriorityConfig
 *
 * @brief		- This function configures interrupt priority
 *
 * @param[in]	- IRQ Interrupt number
 * @param[in]	- IRQ interrupt priority
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority){

	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED) ;

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}

/*****************************************************************
 * @fn			- SPI_IRQHandling
 *
 * @brief		- This function handle interrupts
 *
 * @param[in]	- Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle){

	uint8_t temp1;
	uint8_t temp2;

	/* Check for TXE */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){
		/* Handle TXE */
		spi_txe_interrupt_handle(pHandle);
	}

	/* Check for RXNE */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2){
		/* Handle RXNE */
		spi_rxne_interrupt_handle(pHandle);
	}

	/* Check for OVR flag */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2){
		/* Handle OVR Error */
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

/*****************************************************************
 * @fn			- SPI_PeripheralControl
 *
 * @brief		- This function sets SPI peripheral control
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Enable or Disable command
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);

	} else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);

	}
}

/***************************************************************************************************************************
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
 ***************************************************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	} else{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

/*****************************************************************
 * @fn			- SPI_SSOEConfig
 *
 * @brief		- This function sets SSEO register
 *
 * @param[in]	- Base address of the SPI peripheral
 * @param[in]	- Enable or Disable command
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pSPIx->CR2 |=  (1 << SPI_CR2_SSOE);
	} else{
		pSPIx->CR2 &=  ~(1 << SPI_CR2_SSOE);
	}
}

/*****************************************************************
 * @fn			- SPI_CloseTransmission
 *
 * @brief		- This function close SPI transmission
 *
 * @param[in]	- Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*****************************************************************
 * @fn			- SPI_CloseReception
 *
 * @brief		- This function close SPI reception
 *
 * @param[in]	- Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/*****************************************************************
 * @fn			- SPI_ClearOVRFlag
 *
 * @brief		- This function clears OVR flag
 *
 * @param[in]	- Base address of the SPI peripheral
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){

	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*****************************************************************
 *               Helper functions implementation                 *
 *****************************************************************/
/*****************************************************************
 * @fn			- SPI_ApplicationEventCallback
 *
 * @brief		- Application event callback function
 *
 * @param[in]	- Handle structure
 * @param[in]	- Application event
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent){

	/* This is a week implementation. The application may override this function. */
}

/*****************************************************************
 * @fn			- spi_txe_interrupt_handle
 *
 * @brief		- This function handles TXE in interrupt mode
 *
 * @param[in]	- Pointer to SPI Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){

		/* Load data into data register */
		/* 16 bit */
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else{
		/* 8 bit */
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen){
		/* Tx is zero. Close SPI communication and inform application about it.
		 * Prevents interrupts from setting up of TXE flag. */
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


/*****************************************************************
 * @fn			- spi_rxne_interrupt_handle
 *
 * @brief		- This function handles RXNE in interrupt mode
 *
 * @param[in]	- Pointer to SPI Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		/* Load data from data register into buffer */
		/* 16 bit */
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	} else{
		/* 8 bit */
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;//DBG->Check brackets
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen){
		/* Rx is zero. Close SPI communication and inform application about it.
		 * Prevents interrupts from setting up of RXNE flag. */
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


/*****************************************************************
 * @fn			- spi_ovr_err_interrupt_handle
 *
 * @brief		- This function handles OVR_ERR in
 * 		          interrupt mode
 *
 * @param[in]	- Pointer to SPI Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle){
	uint8_t temp;

	/* Clear OVR flag */
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void)temp;

	/* Inform application */
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}
