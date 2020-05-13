/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: May 11, 2020
 *      Author: Mateus Sousa
 */

#ifndef INC_STM32F401XX_SPI_DRIVER_H_
#define INC_STM32F401XX_SPI_DRIVER_H_

#include "stm32f401xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;				/* SPI Device Mode, 				< possible modes from @SPI_DeviceMode >  		*/
	uint8_t SPI_BusConfig;				/* SPI Bus config, 					< possible bus from @SPI_BusConfig >  	 		*/
	uint8_t SPI_SclkSpeed;				/* SPI clk speed, 					< possible clocks from @SPI_SclkSpeed >  		*/
	uint8_t SPI_DFF;					/* SPI data frame, 					< possible data frames from @SPI_DFF >   		*/
	uint8_t SPI_CPOL;					/* SPI clock polarity, 				< possible clock polarities from @SPI_CPOL >	*/
	uint8_t SPI_CPHA;					/* SPI clock phase, 				< possible clock phases from @SPI_CPHA >		*/
	uint8_t SPI_SSM;					/* SPI Software slave management, 	< possible soft. slave manag. from @SPI_SSM >	*/
}SPI_Config_t;


/*
 *Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;   		/* This holds the base address of SPIx(x:0,1,2) peripheral */
	SPI_Config_t 	SPIConfig;

}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 * SPI possible modes
 */
#define SPI_DEVICE_MODE_SLAVE     0
#define SPI_DEVICE_MODE_MASTER    1

/*
 * @SPI_BusConfig
 * SPI configuration types
 */
#define SPI_BUS_CONFIG_FD                1		/* Full-Duplex */
#define SPI_BUS_CONFIG_HD                2		/* Half-Duplex */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    3		/* Simplex RX only */

/*
 * @SPI_SclkSpeed
 * SPI clock's speed
 */
#define SPI_SCLK_SPEED_DIV2             0
#define SPI_SCLK_SPEED_DIV4             1
#define SPI_SCLK_SPEED_DIV8             2
#define SPI_SCLK_SPEED_DIV16            3
#define SPI_SCLK_SPEED_DIV32            4
#define SPI_SCLK_SPEED_DIV64            5
#define SPI_SCLK_SPEED_DIV128           6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 * Data frame format for transmission/reception
 */
#define SPI_DFF_8BITS 		0
#define SPI_DFF_16BITS  	1

/*
 * @SPI_CPOL
 * SPI clock polarity
 */
#define SPI_CPOL_LOW 		0
#define SPI_CPOL_HIGH 		1

/*
 * @SPI_CPHA
 * SPI clock phase
 */
#define SPI_CPHA_LOW 		0
#define SPI_CPHA_HIGH 		1

/*
 * @SPI_SSM
 * SPI Software slave management
 */
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1


/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other Peripheral Control APIs
 */

#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
