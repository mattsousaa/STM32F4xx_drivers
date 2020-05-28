/*
 * stm32f401xx_usart_driver.c
 *
 *  Created on: 27 de mai de 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_usart_driver.h"
#include "stm32f401xx_rcc_driver.h"
#include "stm32f401xx_i2c_driver.h"

/*****************************************************************
 * @fn			- USART_PeripheralControl
 *
 * @brief		- This function sets U(S)ART peripheral control
 *
 * @param[in]	- Base address of the U(S)ART peripheral
 * @param[in]	- Enable or Disable command
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pUSARTx->CR1 |= (1 << 13);
	} else{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

/*
 * Peripheral Clock setup
 */
/***********************************************************************************************
 * @fn			- USART_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given U(S)ART port
 *
 * @param[in]	- Base address of the U(S)ART peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 ***********************************************************************************************/
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pUSARTx == USART1){
			USART1_PCCK_EN();
		} else if(pUSARTx == USART2){
			USART2_PCCK_EN();
		} else if(pUSARTx == USART6){
			USART6_PCCK_EN();
		}
	} else{
		if(pUSARTx == USART1){
			USART1_PCCK_DI();
		} else if(pUSARTx == USART2){
			USART2_PCCK_DI();
		} else if(pUSARTx == USART6){
			USART6_PCCK_DI();
		}
	}
}

/************************************************************************
 * @fn			- USART_GetFlagStatus
 *
 * @brief		- This function returns if bit in register is set or not
 *
 * @param[in]	- Base address of the U(S)ART peripheral
 * @param[in]	- Name of flag
 *
 * @return		- Flag status (True/False)
 *
 * @Note		- None
 *
 ************************************************************************/
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName){

    if(pUSARTx->SR & StatusFlagName){
    	return SET;
    }

   return RESET;
}

/*******************************************************************
 * @fn      	      - USART_SetBaudRate
 *
 * @brief             - This function sets U(S)ART Baudrate
 *
 * @param[in]         - Base address of the U(S)ART peripheral
 * @param[in]         - Baud rate value
 *
 * @return            - None
 *
 * @Note              - None
 *
 ********************************************************************/
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate){

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

	//Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6){
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
	} else{
	   PCLKx = RCC_GetPCLK1Value();
	}

	//Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 * BaudRate));
	} else{
	   //over sampling by 16
	   usartdiv = ((25 * PCLKx) / (4 * BaudRate));
	}

	//Calculate the Mantissa part
	M_part = usartdiv/100;

	//Place the Mantissa part in appropriate bit position. refer USART_BRR
	tempreg |= M_part << 4;

	//Extract the fraction part
	F_part = (usartdiv - (M_part * 100));

	//Calculate the final fractional
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){
	  //OVER8 = 1, over sampling by 8
	  F_part = ((( F_part * 8) + 50) / 100) & ((uint8_t)0x07); // 50 is a round factor

	} else{
	   //over sampling by 16
	   F_part = ((( F_part * 16) + 50) / 100) & ((uint8_t)0x0F); // 50 is a round factor
	}

	//Place the fractional part in appropriate bit position. refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;

}

/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn			- USART_Init
 *
 * @brief		- This function initialize USART peripherals
 *
 * @param[in]	- Pointer to USART Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_Init(USART_Handle_t *pUSARTHandle){

	//Temporary variable
	uint32_t tempreg = 0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	 USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
		//Implement the code to enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
		//Implement the code to enable the Transmitter bit field
		tempreg |= (1 << USART_CR1_TE);

	} else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	}

    //Implement the code to configure the Word length configuration item
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

    //Configuration of parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		//Implement the code to enale the parity control
		tempreg |= (1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	} else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
		//Implement the code to enable the parity control
	    tempreg |= (1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity
	    tempreg |= (1 << USART_CR1_PS);

	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg = 0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg = 0;

	//Configuration of USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		//Implement the code to enable CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);


	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
		//Implement the code to enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);

	} else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);

}

/*****************************************************************
 * @fn			- USART_DeInit
 *
 * @brief		- This function de-initialize U(S)ART peripherals
 *
 * @param[in]	- Base address of the U(S)ART peripheral
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_DeInit(USART_RegDef_t *pUSARTx){

	if(pUSARTx == USART1){
		USART1_REG_RESET();
	} else if(pUSARTx == USART2){
		USART2_REG_RESET();
	} else if(pUSARTx == USART6){
		USART6_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint16_t *pdata;

   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++){
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

		//Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				//No parity is used in this transfer , so 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			} else{
				//Parity bit is used in this transfer . so 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		} else{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));

}

/******************************************************************
 * @fn      	      - USART_ReceiveData
 *
 * @brief             - This function receives data over U(S)ART
 *
 * @param[in]         - Pointer to U(S)ART Handle Structure
 * @param[in]         - Pointer to Rx Buffer
 * @param[in]         - Length of Rx Buffer
 *
 * @return            - None
 *
 * @Note 	      	  - None
 *
 *****************************************************************/
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len){
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++){
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			//We are going to receive 9bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				//No parity is used , so all 9bits will be of user data

				//read only first 9 bits so mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			} else{
				//Parity is used, so 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
				 pRxBuffer++;
			}
		} else{
			//We are going to receive 8bit data in a frame

			//Now, check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			} else{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}

			//Now , increment the pRxBuffer
			pRxBuffer++;
		}
	}
}

/*******************************************************************************
 * @fn      	      - USART_SendDataInterrupt
 *
 * @brief             - This function sends data over U(S)ART in interrupt mode
 *
 * @param[in]         - Pointer to U(S)ART Handle Structure
 * @param[in]         - Pointer to Tx Buffer
 * @param[in]         - Length of Tx Buffer
 *
 * @return            - Tx state
 *
 * @Note 	          - None
 *
 *******************************************************************************/
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len){

	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX){
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);

	}

	return txstate;
}

/**********************************************************************************
 * @fn      	      - USART_ReceiveDataInterrupt
 *
 * @brief             - This function receives data over U(S)ART in interrupt mode
 *
 * @param[in]         - Pointer to U(S)ART Handle Structure
 * @param[in]         - Pointer to Rx Buffer
 * @param[in]         - Length of Rx Buffer
 *
 * @return            - Rx state
 *
 * @Note 	      	  - None
 *
 **********************************************************************************/
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len){

	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX){
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;
}

/*****************************************************************
 * @fn			- USART_ClearFlag
 *
 * @brief		- This function clears status flag
 *
 * @param[in]	- Base address of the U(S)ART peripheral
 * @param[in]	- Name of flag
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName){

	pUSARTx->SR &= ~(StatusFlagName);

}

/*
 * IRQ Configuration and ISR handling
 */
/*****************************************************************
 * @fn			- USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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
 * @fn			- USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	//1. first lets find out the IPR register
	uint8_t iprx = IRQNumber / 4;				//the corresponding IPR number
	uint8_t iprx_section  = IRQNumber % 4;		//the byte offset of the required Priority field

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //only 4 bits are available

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);

}