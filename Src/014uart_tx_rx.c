/*
 * 014uart_tx_rx.c
 *
 *  Created on: 28 de mai de 2020
 *      Author: Mateus Sousa
 */

#include <stdio.h>
#include <string.h>
#include "stm32f401xx.h"
#include "stm32f401xx_i2c_driver.h"
#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_usart_driver.h"

char msg1[1024] = "UART Tx testing...\n\r";
char msg2[1024] = "The message ended! \n\r";
char rcv_buf[32];
char rcv_print[32];

USART_Handle_t usart2_handle;

void USART2_Init(void){

	usart2_handle.pUSARTx = USART1;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	usart2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

	USART_Init(&usart2_handle);
}

void USART2_GPIOInit(void){

	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOA;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	//USART2 TX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_9;
	GPIO_Init(&usart_gpios);

	//USART2 RX
	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&usart_gpios);

}

void GPIO_ButtonInit(void){

	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

}

void delay(void){
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

int main(void){

	GPIO_ButtonInit();

	USART2_GPIOInit();

    USART2_Init();

    USART_PeripheralControl(USART1, ENABLE);

    while(1){

    	//wait till button is pressed
    	while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4)){};

   		//to avoid button de-bouncing related issues 200ms of delay
   		delay();

   		USART_SendData(&usart2_handle,(uint8_t*)msg1, strlen(msg1));

   		uint8_t i = 0;

   		do{
   			USART_ReceiveData(&usart2_handle, (uint8_t*)rcv_buf, 1);
   			rcv_print[i] = *rcv_buf;
   			i++;
   		} while(*rcv_buf != '\r');

   		rcv_print[i+1] = '\0';

   		USART_SendData(&usart2_handle, (uint8_t*)msg2, strlen(msg2));
   		USART_SendData(&usart2_handle, (uint8_t*)rcv_print, strlen(rcv_print));

    }
}


