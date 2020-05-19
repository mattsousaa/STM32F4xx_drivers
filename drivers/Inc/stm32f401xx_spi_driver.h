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
	uint8_t 		*pTxBuffer; 	/* To store the app. Tx buffer address  */
	uint8_t 		*pRxBuffer;		/* To store the app. Rx buffer address  */
	uint32_t 		TxLen;			/* To store Tx len 						*/
	uint32_t 		RxLen;			/* To store Tx len 						*/
	uint8_t 		TxState;		/* To store Tx state 					*/
	uint8_t 		RxState;		/* To store Rx state 					*/

}SPI_Handle_t;

/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   		1
#define SPI_EVENT_RX_CMPLT   		2
#define SPI_EVENT_OVR_ERR    		3
#define SPI_EVENT_CRC_ERR    		4

/*
 * @SPI_DeviceMode
 * SPI possible modes
 */
#define SPI_DEVICE_MODE_SLAVE     	0
#define SPI_DEVICE_MODE_MASTER    	1

/*
 * @SPI_BusConfig
 * SPI configuration types
 */
#define SPI_BUS_CONFIG_FD                	1		/* SPI Full-duplex bus configuration 		  */
#define SPI_BUS_CONFIG_HD                	2		/* SPI Half-duplex bus configuration 		  */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY    	3		/* SPI Simplex bux receive only configuration */

/*
 * @SPI_SclkSpeed
 * SPI clock's speed
 */
#define SPI_SCLK_SPEED_DIV2			0 		/* Baud rate control: fPCLK/2   */
#define SPI_SCLK_SPEED_DIV4			1 		/* Baud rate control: fPCLK/4   */
#define SPI_SCLK_SPEED_DIV8			2 		/* Baud rate control: fPCLK/8   */
#define SPI_SCLK_SPEED_DIV16		3 		/* Baud rate control: fPCLK/16  */
#define SPI_SCLK_SPEED_DIV32		4 		/* Baud rate control: fPCLK/32  */
#define SPI_SCLK_SPEED_DIV64		5 		/* Baud rate control: fPCLK/64  */
#define SPI_SCLK_SPEED_DIV128		6 		/* Baud rate control: fPCLK/128 */
#define SPI_SCLK_SPEED_DIV256		7 		/* Baud rate control: fPCLK/256 */

/*
 * @SPI_DFF
 * Data frame format for transmission/reception
 */
#define SPI_DFF_8BITS 		0			/* 8-bit DFF is selected for transmission/reception  */
#define SPI_DFF_16BITS  	1			/* 16-bit DFF is selected for transmission/reception */

/*
 * @SPI_CPOL
 * SPI clock polarity
 */
#define SPI_CPOL_LOW 		0			/* Clock polarity to 1 when IDLE */
#define SPI_CPOL_HIGH 		1			/* Clock polarity to 0 when IDLE */

/*
 * @SPI_CPHA
 * SPI clock phase
 */
#define SPI_CPHA_LOW		0 			/* First clock transition <=> First data capture edge  */
#define SPI_CPHA_HIGH		1 			/* Second clock transition <=> First data capture edge */

/*
 * @SPI_SSM
 * SPI Software slave management
 */
#define SPI_SSM_DI			0			/* Software slave management enabled  */
#define SPI_SSM_EN			1			/* Software slave management disabled */

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   	(1 << SPI_SR_BSY)

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * Data send and receive in Interrupt mode
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Length);

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral APIs
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

#endif /* INC_STM32F401XX_SPI_DRIVER_H_ */
