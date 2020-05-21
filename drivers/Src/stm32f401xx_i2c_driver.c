/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: 19 de mai de 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

uint32_t RCC_GetPLLOutputClock();

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_Prescaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr){

	SlaveAddr = SlaveAddr << 1;		// Makes space for read/write bit
	SlaveAddr &= ~(1); 				// SlaveAddr is Slave address + r/nw bit=0 (write)
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx){
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

uint32_t RCC_GetPCLK1Value(void){

	uint32_t pclk1, SystemClk;;

	uint8_t clksrc, temp, ahbp, apb1;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0){
		SystemClk = 16000000;
	} else if(clksrc == 1){
		SystemClk = 8000000;
	} else if(clksrc == 2){
		SystemClk = RCC_GetPLLOutputClock();
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

		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1 ;

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
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr){

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

	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

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

	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}



