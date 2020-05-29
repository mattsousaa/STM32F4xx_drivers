/*
 * stm32f401xx.h
 *
 *  Created on: May 6, 2020
 *      Author: Mateus Sousa
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details **********************************
 *
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)		/* Base address of Interrupt Set-enable Register 0 */
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)		/* Base address of Interrupt Set-enable Register 1 */
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)		/* Base address of Interrupt Set-enable Register 2 */
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)		/* Base address of Interrupt Set-enable Register 3 */

/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			((__vo uint32_t*)0xE000E180)		/* Base address of Interrupt Clear-enable Register 0 */
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)		/* Base address of Interrupt Clear-enable Register 1 */
#define NVIC_ICER2  		((__vo uint32_t*)0xE000E188)		/* Base address of Interrupt Clear-enable Register 2 */
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)		/* Base address of Interrupt Clear-enable Register 3 */

/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4 	/* The number of priority bits implemented is specific to the microcontroller;
 	 	 	 	 	 	 	 	 	 	   In ST's case is actually 4. For instance, in TI's microcontroller, this value is 3.*/

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
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)		/* Base address of I2C2 */
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)		/* Base address of I2C3 */

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)		/* Base address of SPI2 */
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)		/* Base address of SPI3 */

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)		/* Base address of USART2 */

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)		/* Base address of EXTI */

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)		/* Base address of SPI1 */
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)		/* Base address of SPI4 */

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)		/* Base address of USART1 */
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)		/* Base address of USART6 */

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)		/* Base address of SYSCFG */


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
  __vo uint32_t DCKCFGR;       /* RCC Dedicated Clocks Configuration Register					Address offset: 0x8C */

}RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /* Interrupt mask register,						Address offset: 0x00 */
	__vo uint32_t EMR;    /* Event mask register,           				Address offset: 0x04 */
	__vo uint32_t RTSR;   /* Rising trigger selection register,				Address offset: 0x08 */
	__vo uint32_t FTSR;   /* Falling trigger selection register,			Address offset: 0x0C */
	__vo uint32_t SWIER;  /* Software interrupt event register,				Address offset: 0x10 */
	__vo uint32_t PR;     /* Pending register,                  			Address offset: 0x14 */

}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;       	/* SYSCFG memory remap register,                   				Address offset: 0x00      */
	__vo uint32_t PMC;         		/* SYSCFG peripheral mode configuration register,				Address offset: 0x04      */
	__vo uint32_t EXTICR[4];    	/* SYSCFG external interrupt configuration register 1...4,		Address offset: 0x08-0x14 */
	uint32_t      RESERVED1[2]; 	/* Reserved, 0x18-0x1C    																  */
	__vo uint32_t CMPCR;        	/* Compensation cell control register,	   			     		Address offset: 0x20      */
	uint32_t      RESERVED2[2]; 	/* Reserved, 0x24-0x28 	    															  */
}SYSCFG_RegDef_t;

/*
 * peripheral register definition structure for I2C
 */
typedef struct
{
  __vo uint32_t CR1;        /* I2C Control register 1,     							Address offset: 0x00 */
  __vo uint32_t CR2;        /* I2C Control register 2,  							Address offset: 0x04 */
  __vo uint32_t OAR1;       /* I2C Own address register 1,   						Address offset: 0x08 */
  __vo uint32_t OAR2;       /* I2C Own address register 2,   						Address offset: 0x0C */
  __vo uint32_t DR;         /* I2C Data register, 									Address offset: 0x10 */
  __vo uint32_t SR1;        /* I2C Status register 1,  								Address offset: 0x14 */
  __vo uint32_t SR2;        /* I2C Status register 2,  								Address offset: 0x18 */
  __vo uint32_t CCR;        /* I2C Clock control register,  						Address offset: 0x1C */
  __vo uint32_t TRISE;      /* I2C TRISE register,   								Address offset: 0x20 */
  __vo uint32_t FLTR;       /* I2C FLTR register,  									Address offset: 0x24 */
}I2C_RegDef_t;

/*
 * peripheral register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /* SPI control register 1,					Address offset: 0x00 */
	__vo uint32_t CR2;        /* SPI control register 2,					Address offset: 0x04 */
	__vo uint32_t SR;         /* SPI status register,    					Address offset: 0x08 */
	__vo uint32_t DR;         /* SPI data register,							Address offset: 0x0C */
	__vo uint32_t CRCPR;      /* SPI CRC polynomial register,     			Address offset: 0x10 */
	__vo uint32_t RXCRCR;     /* SPI RX CRC register,						Address offset: 0x14 */
	__vo uint32_t TXCRCR;     /* SPI TX CRC register,     					Address offset: 0x18 */
	__vo uint32_t I2SCFGR;    /* SPI_I2S configuration register,    		Address offset: 0x1C */
	__vo uint32_t I2SPR;      /* SPI_I2S prescaler register,   				Address offset: 0x20 */
}SPI_RegDef_t;

/*
 * peripheral register definition structure for USART
 */
typedef struct
{
	__vo uint32_t SR;         /*  											Address offset: 0x00 */
	__vo uint32_t DR;         /*     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*    										Address offset: 0x10 */
	__vo uint32_t CR3;        /*     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*     										Address offset: 0x18 */
}USART_RegDef_t;

/*
 * peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)
 */

#define GPIOA		((GPIO_RegDef_t*) GPIOA_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port A */
#define GPIOB		((GPIO_RegDef_t*) GPIOB_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port B */
#define GPIOC		((GPIO_RegDef_t*) GPIOC_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port C */
#define GPIOD		((GPIO_RegDef_t*) GPIOD_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port D */
#define GPIOE		((GPIO_RegDef_t*) GPIOE_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port E */
#define GPIOH		((GPIO_RegDef_t*) GPIOH_BASEADDR)			/* Base address typecasted to GPIO_RegDef_t port H */

#define RCC			((RCC_RegDef_t*) RCC_BASEADDR)				/* Base address typecasted to RCC_RegDef_t    */
#define EXTI		((EXTI_RegDef_t*) EXTI_BASEADDR)			/* Base address typecasted to EXTI_RegDef_t   */
#define SYSCFG		((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)		/* Base address typecasted to SYSCFG_RegDef_t */

#define SPI1		((SPI_RegDef_t*) SPI1_BASEADDR)				/* Base address typecasted to SPI_RegDef_t SPI1 */
#define SPI2		((SPI_RegDef_t*) SPI2_BASEADDR)				/* Base address typecasted to SPI_RegDef_t SPI2 */
#define SPI3		((SPI_RegDef_t*) SPI3_BASEADDR)				/* Base address typecasted to SPI_RegDef_t SPI3 */
#define SPI4		((SPI_RegDef_t*) SPI4_BASEADDR)				/* Base address typecasted to SPI_RegDef_t SPI4 */

#define I2C1  		((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2  		((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3  		((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1  	((USART_RegDef_t*) USART1_BASEADDR)
#define USART2  	((USART_RegDef_t*) USART2_BASEADDR)
#define USART6 		((USART_RegDef_t*) USART6_BASEADDR)

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
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART6_PCCK_EN() (RCC->APB2ENR |= (1 << 5))

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 * Macros to reset SPIx peripherals
 */
#define USART1_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 4));  (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()	do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART6_REG_RESET()	do{ (RCC->APB2RSTR |= (1 << 5));  (RCC->APB2RSTR &= ~(1 << 5)); }while(0)

/*
 * This macro returns a code(between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
								        (x == GPIOE) ? 4 :\
								        (x == GPIOH) ? 7 :0 )

/*
 * IRQ(Interrupt Request) Numbers of STM32F401xx MCU
 * NOTE: update these macros with valid values according to your MCU
 * TODO: You may complete this list for other peripherals
 */
#define IRQ_NO_EXTI0 			6
#define IRQ_NO_EXTI1 			7
#define IRQ_NO_EXTI2 			8
#define IRQ_NO_EXTI3 			9
#define IRQ_NO_EXTI4 			10
#define IRQ_NO_EXTI9_5 			23
#define IRQ_NO_EXTI15_10 		40

#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2         	36
#define IRQ_NO_SPI3         	51
#define IRQ_NO_SPI4         	84

#define IRQ_NO_I2C1_EV     		31
#define IRQ_NO_I2C1_ER     		32
#define IRQ_NO_I2C2_EV     		33
#define IRQ_NO_I2C2_ER     		34
#define IRQ_NO_I2C3_EV     		79
#define IRQ_NO_I2C3_ER     		80

#define IRQ_NO_USART1	    	37
#define IRQ_NO_USART2	    	38
#define IRQ_NO_USART6	    	71

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRIO0			0			/* Highest priority */
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15			/* Lowest priority */

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
#define FLAG_SET			SET
#define FLAG_RESET			RESET

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	 10
#define SPI_CR1_DFF     			 	 11
#define SPI_CR1_CRCNEXT   			 	 12
#define SPI_CR1_CRCEN   			 	 13
#define SPI_CR1_BIDIOE     			 	 14
#define SPI_CR1_BIDIMODE      			 15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			 0
#define SPI_CR2_TXDMAEN				 	 1
#define SPI_CR2_SSOE				 	 2
#define SPI_CR2_FRF						 4
#define SPI_CR2_ERRIE					 5
#define SPI_CR2_RXNEIE				 	 6
#define SPI_CR2_TXEIE					 7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						 0
#define SPI_SR_TXE				 		 1
#define SPI_SR_CHSIDE				 	 2
#define SPI_SR_UDR					 	 3
#define SPI_SR_CRCERR				 	 4
#define SPI_SR_MODF					 	 5
#define SPI_SR_OVR					 	 6
#define SPI_SR_BSY					 	 7
#define SPI_SR_FRE					 	 8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	 15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					 14
#define I2C_CCR_FS  				 	 15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */
#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#endif /* INC_STM32F401XX_H_ */
