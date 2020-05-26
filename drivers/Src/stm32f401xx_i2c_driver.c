/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: 19 de mai de 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_gpio_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

uint32_t RCC_GetPLLOutputClock();

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_Prescaler[4] = {2, 4, 8, 16};

uint32_t RCC_GetPCLK1Value(void){

	uint32_t pclk1, SystemClk;

	uint8_t clksrc, temp, ahbp, apb1;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){
		SystemClk = 16000000;

	} else if(clksrc == 1){
		SystemClk = 8000000;

	} else if(clksrc == 2){
		//SystemClk = RCC_GetPLLOutputClock();
	}

	//AHB
	temp = ((RCC->CFGR >> 4) & 0xF);

	if(temp < 8){
		ahbp = 1;
	} else{
		ahbp = AHB_Prescaler[temp - 8];
	}

	//APB1
	temp = ((RCC->CFGR >> 10) & 0x7);

	if(temp < 4){
		apb1 = 1;
	} else{
		apb1 = APB1_Prescaler[temp - 4];
	}

	pclk1 = (SystemClk / ahbp)/apb1;

	return pclk1;
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == I2C_ACK_ENABLE){
		//enable the ack
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	} else{
		//disable the ack
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*****************************************************************
 * @fn			- I2C_GenerateStopCondition
 *
 * @brief		- Generate stop condition for I2C
 *
 * @param[in]	- Base address of the I2C peripheral
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void teste(){

	GPIO_Handle_t GpioLed;

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);

	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_5);
}

/*
 * Peripheral Clock setup
 */
/*********************************************************************************************
 * @fn			- I2C_PeriClockControl
 *
 * @brief		- This function enables or disables peripheral clock for the given I2C port
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Macros: Enable or Disable
 *
 * @return		- None
 *
 * @Note		- None
 *
 *********************************************************************************************/
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		} else if (pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	} else{
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		} else if (pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if (pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

/*****************************************************************
 * @fn			- I2C_PeripheralControl
 *
 * @brief		- This function sets I2C peripheral control
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Enable or Disable command
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

/*
 * Init and De-Init
 */
/*****************************************************************
 * @fn			- I2C_Init
 *
 * @brief		- This function initialize I2C peripherals
 *
 * @param[in]	- Pointer to I2C Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void I2C_Init(I2C_Handle_t *pI2CHandle){

	uint32_t tempreg = 0;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << 10);

	pI2CHandle->pI2Cx->CR1 |= tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);													/* Should always be kept at 1 by software */
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){
		//mode is standard mode

		/* T_high(scl) = CCR * T_pclk1
		 * T_low(scl) = CCR * T_pclk1
		 * T_high + T_low = T_scl
		 * T_scl = 2 * CCR * T_pclk1
		 * CCR = T_scl / (2 * T_pclk1) [time domain]
		 * CCR = F_pclk1 / (2 * F_scl) [frequency domain] */

		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);										/* Only 12 bits */

	} else{
		//mode is fast mode

		tempreg |= (1 << 15);												/* F/S: I2C master mode selection (FM) */
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);			/* DUTY: Fm mode duty cycle */

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){

			/* T_high(scl) = CCR * T_pclk1
			 * T_low(scl) = 2 * CCR * T_pclk1
			 * T_high + T_low = T_scl
			 * T_scl = 3 * CCR * T_pclk1
			 * CCR = T_scl / (3 * T_pclk1) [time domain]
			 * CCR = F_pclk1 / (3 * F_scl) [frequency domain] */

			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		} else{

			/* T_high(scl) = 9 * CCR * T_pclk1
			* T_low(scl) = 16 * CCR * T_pclk1
			* T_high + T_low = T_scl
			* T_scl = 25 * CCR * T_pclk1
			* CCR = T_scl / (25 * T_pclk1) [time domain]
			* CCR = F_pclk1 / (25 * F_scl) [frequency domain] */

			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		tempreg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR |= tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM){

		/* Rise time of both SDA and SCL signals (Standard-mode)
		 * Maximum rise time: 1000 ns or 1 us
		 * */

		/* TRISE = (T_rise / T_pclk1) + 1
		 * TRISE = (T_rise * F_pclk1) + 1 */

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;

	} else{
		/* Rise time of both SDA and SCL signals (Standard-mode)
		 * Maximum rise time: 300 ns or 300 us
		 * */

		/* TRISE = (T_rise / T_pclk1) + 1
		 * TRISE = (T_rise * F_pclk1) + 1 */

		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/*****************************************************************
 * @fn			- I2C_DeInit
 *
 * @brief		- This function de-initialize I2C peripherals
 *
 * @param[in]	- Base address of the I2C peripheral
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void I2C_DeInit(I2C_RegDef_t *pI2Cx){

	if(pI2Cx == I2C1){
		I2C1_REG_RESET();
	} else if(pI2Cx == I2C2){
		I2C2_REG_RESET();
	} else if(pI2Cx == I2C3){
		I2C3_REG_RESET();
	}
}

/*************************************************************************
 * @fn			- I2C_GetFlagStatus
 *
 * @brief		- This function returns if bit in register is set or not
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Name of flag
 *
 * @return		- Flag status (True/False)
 *
 * @Note		- None
 *
 *************************************************************************/
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName){

	if(pI2Cx->SR1 & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*****************************************************************
 * @fn			- I2C_MasterSendData
 *
 * @brief		- I2C Master sends data to slaves
 *
 * @param[in]	- Pointer to I2C Handle structure
 * @param[in]	- Pointer to transmit buffer
 * @param[in]	- Length of transmit buffer
 * @param[in]	- Slave address
 * @param[in]	- State of status register
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)

	I2C_ClearADDRFlag(pI2CHandle);

	//6. send the data until len becomes 0

	while(Len > 0){
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //Wait till TXE is set
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}

	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// 	 Note: TXE=1, BTF=1, means that both SR and DR are empty and next transmission should begin
	// 	 when BTF=1 SCL will be stretched (pulled to LOW)

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(Sr == I2C_DISABLE_SR)	//Repeated start (Sr)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/*****************************************************************
 * @fn			- I2C_MasterReceiveData
 *
 * @brief		- I2C Master receive data from slaves
 *
 * @param[in]	- Pointer to I2C Handle structure
 * @param[in]	- Pointer to receive buffer
 * @param[in]	- Length of receive buffer
 * @param[in]	- Slave address
 * @param[in]	-
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){

	//1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//procedure to read only 1 byte from slave
	if(Len == 1){
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR)	//Repeated start (Sr)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}

	 //procedure to read data from slave when Len > 1
	if(Len > 1){

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//read the data until Len becomes zero
		for(uint32_t i = Len; i > 0; i--){
			//wait until RXNE becomes 1
			while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

			if(i == 2){ //if last 2 bytes are remaining
				//Disable Acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR)	//Repeated start (Sr)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from data register into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;

		}
	}

	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}
}

/*****************************************************************
 * @fn			- I2C_CloseReceiveData
 *
 * @brief		- Closing I2C communication when data is received
 *
 * @param[in]	- Pointer to I2C Handle structure
 *
 * @return		- None
 *
 * @Note		- Disabling all interrupts
 *
 *****************************************************************/
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle){

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/*****************************************************************
 * @fn			- I2C_CloseSendData
 *
 * @brief		- Closing I2C communication when data is sent
 *
 * @param[in]	- Pointer to I2C Handle structure
 *
 * @return		- None
 *
 * @Note		- Disabling all interrupts
 *
 *****************************************************************/
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle){

	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
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
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){

		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)){

		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; 					//Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle){

	//Interrupt handling for both master and slave mode of a device

	teste();

	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	// For setting it, see Table 71 on reference manual

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);		//Check the SB (start bit) flag
	//1. Handle for interrupt generated by SB event
	// 	 Note : SB flag is only applicable in Master mode

	if(temp1 && temp3){
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//In this block lets executed the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode  : Address matched with own address
	if(temp1 && temp3){
		// interrupt is generated because of ADDR event
		I2C_ClearADDRFlag(pI2CHandle);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3){
		//BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			//make sure that TXE is also set
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				//BTF, TXE = 1
				if(pI2CHandle->TxLen == 0){		// Make sure that Tx Lenght is equal to zero
					//1. generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR){
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		} else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			; //nothing to do here
		}

	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	// The below code block will not be executed by the master since STOPF will not set in master mode
	if(temp1 && temp3){
		//STOF flag is set
		//Clear the STOPF ( i.e 1) read SR1 (already done above) 2) Write to CR1

		pI2CHandle->pI2Cx->CR1 |= 0x0000;	// Do not modify the register in write operation

		//Notify the application that STOP is detected or generated by the master
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3){
		//TXE flag is set
		//Check for device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			//We have to do the data transmission
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		} else{
			//slave
			//make sure that the slave is really in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3){
		//RXNE flag is set
		//check device mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			//The device is master
			//RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		} else{
			//slave
			//make sure that the slave is really in receiver mode
			if(! (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*****************************************************************
 * @fn			- I2C_ER_IRQHandling
 *
 * @brief		- Interrupt handling for different I2C errors
 *
 * @param[in]	- Pointer to I2C Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle){

	teste();

	uint32_t temp1, temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2){
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);

	if(temp1 && temp2){
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);

	if(temp1 && temp2){
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);

	if(temp1 && temp2){
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);

	if(temp1 && temp2){
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}
}

/*
 * IRQ Configuration and ISR handling
 */

/*****************************************************************
 * @fn			- I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){

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
 * @fn			- I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){
	//1. first lets find out the IPR register
	uint8_t iprx = IRQNumber / 4;				//the corresponding IPR number
	uint8_t iprx_section  = IRQNumber % 4;		//the byte offset of the required Priority field

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); //only 4 bits are available

	*(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/*****************************************************************
 *               Helper functions implementation                 *
 *****************************************************************/
/*****************************************************************
 * @fn			- I2C_GenerateStartCondition
 *
 * @brief		- Generate start condition for I2C
 *
 * @param[in]	- Base address of the I2C peripheral
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/*******************************************************************************************
 * @fn			- I2C_ExecuteAddressPhaseWrite
 *
 * @brief		- Send the address of the slave with r/nm bit set to r/nw(0) (total 8 bits)
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Slave address
 *
 * @return		- None
 *
 * @Note		- None
 *
 *******************************************************************************************/
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;		// Makes space for read/write bit
	SlaveAddr &= ~(1); 				// SlaveAddr is Slave address + r/nw bit=0 (write)
	pI2Cx->DR = SlaveAddr;
}

/*******************************************************************************************
 * @fn			- I2C_ExecuteAddressPhaseRead
 *
 * @brief		- Send the address of the slave with r/nm bit set to r/nw(1) (total 8 bits)
 *
 * @param[in]	- Base address of the I2C peripheral
 * @param[in]	- Slave address
 *
 * @return		- None
 *
 * @Note		- None
 *
 *******************************************************************************************/
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 1; 						//SlaveAddr is Slave address + r/nw bit=1
	pI2Cx->DR = SlaveAddr;
}

/*****************************************************************
 * @fn			- I2C_ClearAddrFlag
 *
 * @brief		- Clear address flag
 *
 * @param[in]	- Pointer to I2C Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){

	uint32_t dummy_read;
	//check for device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
		//device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize == 1){
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the ADDR flag (read SR1 , read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		} else{
			//clear the ADDR flag (read SR1 , read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	} else{
		//device is in slave mode
		//clear the ADDR flag (read SR1 , read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

/*****************************************************************
 * @fn			- I2C_MasterHandleTXEInterrupt
 *
 * @brief		- Interrupt handling master TXE mode
 *
 * @param[in]	- Pointer to I2C Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle){

	if(pI2CHandle->TxLen > 0){
		//1. load the data in to DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

		//2. decrement the TxLen
		pI2CHandle->TxLen--;

		//3. Increment the buffer address
		pI2CHandle->pTxBuffer++;

	}
}

/*****************************************************************
 * @fn			- I2C_MasterHandleRXNEInterrupts
 *
 * @brief		- Interrupt handling master RXNE mode
 *
 * @param[in]	- Pointer to I2C Handle structure
 *
 * @return		- None
 *
 * @Note		- None
 *
 *****************************************************************/
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle){
	//We have to do the data reception
	if(pI2CHandle->RxSize == 1){
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1){

		if(pI2CHandle->RxLen == 2){
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}
		//read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0){
		//close the I2C data reception and notify the application

		//1. Generate the stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//2. Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}
