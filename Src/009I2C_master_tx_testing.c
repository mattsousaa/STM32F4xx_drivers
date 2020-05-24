/*
 * 009I2C_master_tx_testing.c
 *
 *  Created on: 21 de mai de 2020
 *      Author: Mateus Sousa
 */

#include <string.h>
#include "stm32f401xx_gpio_driver.h"
#include "stm32f401xx_i2c_driver.h"

#define MY_ADDR 	0x61

#define SLAVE_ADDR  0x68

void delay(void){
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C3Handle;

//some data
uint8_t some_data[] = "We are testing I2C master Tx\n";

/*
 * PA8-> SCL // 28
 * PB4-> SDA // 27
 */

void I2C1_GPIOInits(void){

	GPIO_Handle_t SCLPin, SDAPin;

	//SCL
	SCLPin.pGPIOx = GPIOA;
	SCLPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SCLPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	SCLPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SCLPin.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	SCLPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	SCLPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&SCLPin);

	//SDA
	SDAPin.pGPIOx = GPIOB;
	SDAPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SDAPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	SDAPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	SDAPin.GPIO_PinConfig.GPIO_PinAltFunMode = 9;
	SDAPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//sda
	SDAPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SDAPin);

}

void I2C1_Inits(void){

	I2C3Handle.pI2Cx = I2C3;
	I2C3Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C3Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C3Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C3Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C3Handle);

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

int main(void){

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C3, ENABLE);

	while(1){
		//wait till button is pressed
		while( GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_4));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//send some data to the slave
		I2C_MasterSendData(&I2C3Handle, some_data, strlen((char*)some_data), SLAVE_ADDR);

	}
}

