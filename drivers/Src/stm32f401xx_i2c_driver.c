/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: 19 de mai de 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_i2c_driver.h"

uint32_t RCC_GetPLLOutputClock();

uint16_t AHB_Prescaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_Prescaler[4] = {2, 4, 8, 16};

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

/*********************************************************************
 * @fn      		  - I2C_PeriClockControl
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

/*********************************************************************
 * @fn      		  - I2C_PeripheralControl
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
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else{
		pI2Cx->CR1 &= ~(1 << 0);
	}
}

/*********************************************************************
 * @fn      		  - I2C_Init
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

}





