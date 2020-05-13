/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: 6 de mai de 2020
 *      Author: Mateus Sousa Araújo
 */

#ifndef INC_STM32F401XX_GPIO_DRIVER_H_
#define INC_STM32F401XX_GPIO_DRIVER_H_

#include "stm32f401xx.h"

/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/* GPIO number, 			< possible values from @GPIO_PIN_NUMBERS >  */
	uint8_t GPIO_PinMode;			/* GPIO mode, 				< possible values from @GPIO_PIN_MODES >    */
	uint8_t GPIO_PinSpeed;			/* GPIO speed, 				< possible values from @GPIO_PIN_SPEED >    */
	uint8_t GPIO_PinPuPdControl;	/* GPIO Pull up/down, 		< possible values from @GPIO_PIN_PU_PD >    */
	uint8_t GPIO_PinOPType;			/* GPIO output type,  		< possible values from @GPIO_PIN_PP_OD >    */
	uint8_t GPIO_PinAltFunMode;		/* GPIO function mode, 		< possible values from @GPIO_PIN_AF_MODES > */
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;       		/* This holds the base address of the GPIO port to which the pin belongs */
	GPIO_PinConfig_t GPIO_PinConfig;    /* This holds GPIO pin configuration settings */

}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0  		0
#define GPIO_PIN_NO_1  		1
#define GPIO_PIN_NO_2  		2
#define GPIO_PIN_NO_3  		3
#define GPIO_PIN_NO_4  		4
#define GPIO_PIN_NO_5  		5
#define GPIO_PIN_NO_6  		6
#define GPIO_PIN_NO_7  		7
#define GPIO_PIN_NO_8  		8
#define GPIO_PIN_NO_9  		9
#define GPIO_PIN_NO_10  	10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12  	12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14 		14
#define GPIO_PIN_NO_15 		15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2		/* Alternative function */
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4		/* Input falling-edge trigger */
#define GPIO_MODE_IT_RT     5		/* Input rising-edge trigger*/
#define GPIO_MODE_IT_RFT    6		/* Input rising-edge/falling-edge trigger */

/*
 * @GPIO_PIN_PP_OD
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   	0		/* Push-pull type */
#define GPIO_OP_TYPE_OD   	1		/* Open-drain type */

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3

/*
 * @GPIO_PIN_PU_PD
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   			0	/* Non pull up/down pin */
#define GPIO_PIN_PU				1	/* Pull-up pin */
#define GPIO_PIN_PD				2   /* Pull-down pin */

/*
 * @GPIO_PIN_AF_MODES
 * GPIO alternate function modes for port x bit y (y = 0..7 and 8...15)
 */
#define AF0				0		/* system 	 */
#define AF1				1		/* TIM1/TIM2 */
#define AF2				2		/* TIM3..5 	 */
#define AF3				3		/* TIM9..11  */
#define AF4				4		/* I2C1..3 	 */
#define AF5				5		/* SP1..4 	 */
#define AF6				6		/* SPI3 	 */
#define AF7				7		/* USART1..2 */
#define AF8				8		/* USART6 	 */
#define AF9				9		/* I2C2..3 	 */
#define AF10			10		/* OTG_FS 	 */
#define AF11			11		/* none	 	 */
#define AF12			12		/* SDIO 	 */
#define AF13			13		/* none 	 */
#define AF14			14		/* none 	 */
#define AF15			15		/* EVENTOUT  */

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */
