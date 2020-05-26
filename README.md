## STM32F4xx_drivers

Lab assignments from [Udemy](https://www.udemy.com/share/100F3uBkcacF1XR3o=/?xref=E0QfcF9RQH4HSWUuAAcqP1kSWSRM) course [MCU1](https://www.udemy.com/share/101rCkBkcacF1XR3o=/) by [FastBit Embedded Brain Academy](http://fastbitlab.com/). Contains source coude for all exercises done during course, tested and adjusted for [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) board.


## Usage

For this assignments [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board was used, but you can use code and adjust it according to your board.

Src folder contains tests, which covers every major implementation (GPIO, I2C, SPI, USART, IRQs)
The folder structure is described as follows:

|                 |Files                            |Description                  |
|----------------|---------------------------------|-----------------------------|
|**/src**	 |`".c"` files		   	   |Application Layer|
|**/drivers**    |Include and source files  	   |Driver Layer |
|**/drivers/inc**|`"stm32f401xx.h"` <br> `"stm32f401xx_gpio_driver.h"`<br> `"stm32f401xx_i2c_driver.h"` <br> `"stm32f401xx_spi_driver.h"`|-- MCU Specific Header File: It contains all the device specific details <br> -- GPIO Header File <br> -- I2C Header File <br> -- SPI Header File |
|**/drivers/src**|`"stm32f401xx_gpio_driver.c"` <br> `"stm32f401xx_i2c_driver.c"` <br> `"stm32f401xx_spi_driver.c"`|		   -- GPIO Source File <br> -- I2C Source File <br> -- SPI Source File |


## Setup Installation - Ubuntu 18.04
* ##### GCC Toolchain installation (v7.5.0)
  * `$ gcc -v     // check C compiler is installed or not` 
   &nbsp;
   
  * Open terminal and type following command to install GCC\
  `$ sudo apt-get install gcc`
  
* ##### STM32Cube IDE installation (v1.3.0)
  * [Download installation for Linux](https://www.st.com/en/development-tools/stm32cubeide.html) (Debian Linux Installer)
  * Navigate to downloads folder and run following command in terminal\
  `sudo chmod +x script-name-here.sh` This command will set execute permission on installation script.
  * To run installation script run one of the following commands:
    * `sudo ./script-name-here.sh`
    or
    * `sudo sh script-name-here.sh`
    or
    * `sudo bash script-name-here.sh`

* ##### *FPU* warning fix
    Right click on the project -> properties -> expand C/C++ build -> Settings -> Tool settings -> MCU settings
  * `Floating-point unit: None`
  * `Floating-point ABI: Software implementation ( -mfloat-abi=soft )`

* ##### Installing GIT (v2.7.1)
  * Install Git using apt-get:\
   `$ sudo apt-get update`\
   `$ sudo apt-get install git`
   &nbsp;
   
  * Verify the installation was successful by typing:\
  `$ git --version`
  
* ##### Installing GIT-GUI client` GitFiend`(v0.22.8)
  * Download [`GitFiend`](https://gitfiend.com/overview) *.deb* file
  * Navigate to *Downloads* folder and type `sudo apt-get install ./package-name.deb`
  * Setup `GitFiend` and connect it with your repository

&nbsp;
### *Setting up SWV ITM Data Console*

Open *syscalls.c* file and paste following code bellow *Includes*

```c
#define __vo volatile
```

```c
/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;		/* GPIO port pull-up/pull-down register,		Address offset: 0x00 */
	__vo uint32_t OTYPER;          	/* GPIO port output type register,			Address offset: 0x04 */
	__vo uint32_t OSPEEDR;		/* GPIO port output speed register, 			Address offset: 0x08 */
	__vo uint32_t PUPDR;		/* GPIO port pull-up/pull-down register,		Address offset: 0x0C */
	__vo uint32_t IDR;		/* GPIO port input data register,			Address offset: 0x10 */
	__vo uint32_t ODR;		/* GPIO port output data register,	 		Address offset: 0x14 */
	__vo uint32_t BSRR;		/* GPIO port bit set/reset register,			Address offset: 0x18 */
	__vo uint32_t LCKR;		/* GPIO port bit set/reset register,			Address offset: 0x1C */
	__vo uint32_t AFR[2];		/* GPIO alternate function high register,		Address offset: 0x20-0x24 */
}GPIO_RegDef_t;
```


&nbsp;

```c
/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
  __vo uint32_t CR;            /* RCC clock control register,					Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /* RCC PLL configuration register,				Address offset: 0x04 */
  __vo uint32_t CFGR;          /* RCC clock configuration register,				Address offset: 0x08 */
  __vo uint32_t CIR;           /* RCC clock interrupt register,					Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /* RCC AHB1 peripheral reset register,				Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /* RCC AHB2 peripheral reset register,				Address offset: 0x14 */
  uint32_t  	RESERVED0[2];  /* Reserved, 0x18-0X1C                                         	 	     	     */
  __vo uint32_t APB1RSTR;      /* RCC APB1 peripheral reset register,				Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /* RCC APB2 peripheral reset register,				Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /* Reserved, 0x28-0x2C                                             		     */
  __vo uint32_t AHB1ENR;       /* RCC AHB1 peripheral clock enable register,			Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /* RCC AHB2 peripheral clock enable register,    		Address offset: 0x34 */
  uint32_t      RESERVED2[2];  /* Reserved, 0x38-0X3C                                             		     */
  __vo uint32_t APB1ENR;       /* RCC APB1 peripheral clock enable register,     		Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /* RCC APB2 peripheral clock enable register,    		Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /* Reserved, 0x48-0x4C                                             		     */
  __vo uint32_t AHB1LPENR;     /* RCC AHB1 peripheral clock enable in low power mode register,  Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /* RCC AHB2 peripheral clock enable in low power mode register,  Address offset: 0x54 */
  uint32_t      RESERVED4[2];  /* Reserved, 0x58-0X5C                                             		     */
  __vo uint32_t APB1LPENR;     /* RCC APB1 peripheral clock enable in low power mode register,  Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /* RCC APB2 peripheral clock enabled in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /* Reserved, 0x68-0x6C                                             		     */
  __vo uint32_t BDCR;          /* RCC Backup domain control register,    			Address offset: 0x70 */
  __vo uint32_t CSR;           /* RCC clock control & status register,    			Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /* Reserved, 0x78-0x7C                                             		     */
  __vo uint32_t SSCGR;         /* RCC spread spectrum clock generation register,    		Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /* RCC PLLI2S configuration register,    			Address offset: 0x84 */
  uint32_t      RESERVED7;     /* Reserved, 0x88    								     */
  __vo uint32_t DCKCFGR;       /* RCC Dedicated Clocks Configuration Register			Address offset: 0x8C */

}RCC_RegDef_t;

```

&nbsp;

```c
/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /* Interrupt mask register,				Address offset: 0x00 */
	__vo uint32_t EMR;    /* Event mask register,           			Address offset: 0x04 */
	__vo uint32_t RTSR;   /* Rising trigger selection register,			Address offset: 0x08 */
	__vo uint32_t FTSR;   /* Falling trigger selection register,			Address offset: 0x0C */
	__vo uint32_t SWIER;  /* Software interrupt event register,			Address offset: 0x10 */
	__vo uint32_t PR;     /* Pending register,                  			Address offset: 0x14 */

}EXTI_RegDef_t;
```

&nbsp;
## Contributing 

Pull requests are welcome. If you discover any bug/issue feel free to report it.
