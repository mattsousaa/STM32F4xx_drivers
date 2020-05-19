/*
 * stm32f401xx_i2c_driver.c
 *
 *  Created on: 19 de mai de 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_i2c_driver.h"

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





