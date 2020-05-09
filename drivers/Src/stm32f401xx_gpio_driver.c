/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: 6 de mai de 2020
 *      Author: Mateus Sousa
 */

#include "stm32f401xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	} else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		} else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		} else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		} else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		} else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		} else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the GPIO ports
 *
 * @param[in]         - It holds the base address of the GPIO port and GPIO pin configuration settings (Handle structure)
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -	none
 *
 * @Note              - none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0;	// temp register

	//1 . configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// The non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
		temp = 0;
	} else{

	}

	temp = 0;

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	temp = 0;

	//3. configure the pull-up/pull-down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	temp = 0;

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	temp = 0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));

	}
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function deinitializes the GPIO ports
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              -	none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	} else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	} else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	} else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	} else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the input state of a GPIO pin
 *
 * @param[in]         - Base address of the gpio peripheral
 * @param[in]         - Pin number of GPIO
 * @param[in]         -
 *
 * @return            - 0 or 1
 *
 * @Note              - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001 ) ;

	return value;

}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads the value of a entire GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - Value of GPIO port
 *
 * @Note              - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes high or low for an output pin
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number of GPIO
 * @param[in]         - Value of GPIO (High or Low)
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	} else{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes a certain value for a entire GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Desired value of port
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR  = Value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the output state of a GPIO pin
 *
 * @param[in]         - Base address of the GPIO peripheral
 * @param[in]         - Pin number of GPIO
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
