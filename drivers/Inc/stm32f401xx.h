/*
 * stm32f401xx.h
 *
 *  Created on: May 6, 2020
 *      Author: Mateus Sousa
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * Base addresses of Flash and SRAM memory
 */

#define FLASH_BASEADDR				0x08000000U			/* Base address of flash memory */
#define SRAM1_BASEADDR				0x20000000U			/* Base address of SRAM1 memory (96 KB) */
#define ROM_BASEADDR				0x1FFF0000U			/* Base address of ROM memory (36 KB) */
#define SRAM						SRAM1_BASEADDR		/* Base address of SRAM memory */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE 				0x40000000U			/* Base address of peripherals */
#define APB1PERIPH_BASEADDR			PERIPH_BASE			/* Base address of APB1 bus */
#define APB2PERIPH_BASEADDR			0x40010000U			/* Base address of APB2 bus */
#define AHB1PERIPH_BASEADDR			0x40020000U			/* Base address of AHB1 bus */
#define AHB2PERIPH_BASEADDR			0x50000000U			/* Base address of AHB2 bus */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)		/* Base address of GPIOA */
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)		/* Base address of GPIOB */
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)		/* Base address of GPIOC */
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)		/* Base address of GPIOD */
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)		/* Base address of GPIOE */
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)		/* Base address of GPIOH */

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800)		/* Base address of RCC */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)		/* Base address of I2C1 */
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5BFF)		/* Base address of I2C2 */
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5FFF)		/* Base address of I2C3 */

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3BFF)		/* Base address of SPI2 */
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3FFF)		/* Base address of SPI3 */

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x47FF)		/* Base address of USART2 */

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)		/* Base address of EXTI */

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x33FF)		/* Base address of SPI1 */
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x37FF)		/* Base address of SPI4 */

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x13FF)		/* Base address of USART1 */
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x17FF)		/* Base address of USART6 */

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3BFF)		/* Base address of SYSCFG */


/**********************************peripheral register definition structures **********************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * e.g : Number of Registers of SPI peripheral of STM32F4x family of MCUs may be different(more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check your Device RM
 */

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;			/* GPIO port pull-up/pull-down register,		Address offset: 0x00 */
	__vo uint32_t OTYPER;           /* GPIO port output type register,				Address offset: 0x04 */
	__vo uint32_t OSPEEDR;			/* GPIO port output speed register, 			Address offset: 0x08 */
	__vo uint32_t PUPDR;			/* GPIO port pull-up/pull-down register,		Address offset: 0x0C */
	__vo uint32_t IDR;				/* GPIO port input data register,				Address offset: 0x10 */
	__vo uint32_t ODR;				/* GPIO port output data register,	 			Address offset: 0x14 */
	__vo uint32_t BSRR;				/* GPIO port bit set/reset register,			Address offset: 0x18 */
	__vo uint32_t LCKR;				/* GPIO port bit set/reset register,			Address offset: 0x1C */
	__vo uint32_t AFR[2];		 	/* GPIO alternate function high register,		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /* RCC clock control register,									Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /* RCC PLL configuration register,								Address offset: 0x04 */
  __vo uint32_t CFGR;          /* RCC clock configuration register,								Address offset: 0x08 */
  __vo uint32_t CIR;           /* RCC clock interrupt register,									Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /* RCC AHB1 peripheral reset register,							Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /* RCC AHB2 peripheral reset register,							Address offset: 0x14 */
  uint32_t  	RESERVED0[2];  /* Reserved, 0x18-0X1C                                         	 					 */
  __vo uint32_t APB1RSTR;      /* RCC APB1 peripheral reset register,							Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /* RCC APB2 peripheral reset register,							Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /* Reserved, 0x28-0x2C                                             					 */
  __vo uint32_t AHB1ENR;       /* RCC AHB1 peripheral clock enable register,					Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /* RCC AHB2 peripheral clock enable register,    				Address offset: 0x34 */
  uint32_t      RESERVED2[2];  /* Reserved, 0x38-0X3C                                             					 */
  __vo uint32_t APB1ENR;       /* RCC APB1 peripheral clock enable register,     				Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /* RCC APB2 peripheral clock enable register,    				Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /* Reserved, 0x48-0x4C                                             					 */
  __vo uint32_t AHB1LPENR;     /* RCC AHB1 peripheral clock enable in low power mode register,  Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /* RCC AHB2 peripheral clock enable in low power mode register,  Address offset: 0x54 */
  uint32_t      RESERVED4[2];  /* Reserved, 0x58-0X5C                                             					 */
  __vo uint32_t APB1LPENR;     /* RCC APB1 peripheral clock enable in low power mode register,  Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /* RCC APB2 peripheral clock enabled in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /* Reserved, 0x68-0x6C                                             					 */
  __vo uint32_t BDCR;          /* RCC Backup domain control register,    						Address offset: 0x70 */
  __vo uint32_t CSR;           /* RCC clock control & status register,    						Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /* Reserved, 0x78-0x7C                                             					 */
  __vo uint32_t SSCGR;         /* RCC spread spectrum clock generation register,    			Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /* RCC PLLI2S configuration register,    						Address offset: 0x84 */
  uint32_t      RESERVED7;     /* Reserved, 0x88    																 */
  __vo uint32_t DCKCFGR;       /* RCC Dedicated Clocks Configuration Register    				Address offset: 0x8C */

} RCC_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port A */
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port B */
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port C */
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port D */
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port E */
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port H */

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)				/* Base address typecasted to RCC_RegDef_t */

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 0))			/* Enable GPIOA clock */
#define GPIOB_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 1))			/* Enable GPIOB clock */
#define GPIOC_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 2))			/* Enable GPIOC clock */
#define GPIOD_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 3))			/* Enable GPIOD clock */
#define GPIOE_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 4))			/* Enable GPIOE clock */
#define GPIOH_PCLK_EN()    	(RCC->AHB1ENR |= (1 << 7))			/* Enable GPIOH clock */

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()    	(RCC->APB1ENR |= (1 << 21))			/* Enable I2C1 clock */
#define I2C2_PCLK_EN()    	(RCC->APB1ENR |= (1 << 22))			/* Enable I2C2 clock */
#define I2C3_PCLK_EN()    	(RCC->APB1ENR |= (1 << 23))			/* Enable I2C3 clock */

/*
 * Clock Enable Macros for SPIx peripheralsbu
 */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1 << 12))				/* Enable SP1 clock */
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1 << 14))				/* Enable SP2 clock */
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1 << 15))				/* Enable SP3 clock */
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1 << 13))				/* Enable SP4 clock */

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()       do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))				/* Enable USART1 clock */
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))			/* Enable USART2 clock */
#define USART6_PCCK_EN() (RCC->APB2ENR |= (1 << 5))				/* Enable USART6 clock */

/*
 * Clock Enable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))			/* Enable SYSCFG clock */

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0))				/* Disable GPIOA clock */
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1))				/* Disable GPIOB clock */
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 2))				/* Disable GPIOC clock */
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 3))				/* Disable GPIOD clock */
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 4))				/* Disable GPIOE clock */
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &= ~(1 << 7))				/* Disable GPIOH clock */

/*
 * Clock Disable Macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 21))			/* Disable I2C1 clock */
#define I2C2_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 22))			/* Disable I2C2 clock */
#define I2C3_PCLK_DI()   (RCC->APB1ENR &= ~(1 << 23))			/* Disable I2C3 clock */

/*
 * Clock Disable Macros for SPIx peripheralsbu
 */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1 << 12))				/* Disable SP1 clock */
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 14))				/* Disable SP2 clock */
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 15))				/* Disable SP3 clock */
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1 << 13))				/* Disable SP4 clock */

/*
 * Clock Disable Macros for USARTx peripherals
 */

#define USART1_PCCK_DI() (RCC->APB2ENR &= ~(1 << 4))			/* Disable USART1 clock */
#define USART2_PCCK_DI() (RCC->APB1ENR &= ~(1 << 17))			/* Disable USART2 clock */
#define USART6_PCCK_DI() (RCC->APB2ENR &= ~(1 << 5))			/* Disable USART6 clock */

/*
 * Clock Disable Macros for SYSCFG peripheral
 */

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1 << 14))			/* Disable SYSCFG clock */

/*
 * Some generic macros
 */

#define ENABLE				1
#define DISABLE				0
#define HIGH				ENABLE
#define LOW					DISABLE
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


#endif /* INC_STM32F401XX_H_ */
