## STM32F4xx_drivers

Lab assignments from [Udemy](https://www.udemy.com/share/100F3uBkcacF1XR3o=/?xref=E0QfcF9RQH4HSWUuAAcqP1kSWSRM) course [MCU1](https://www.udemy.com/share/101rCkBkcacF1XR3o=/) by [FastBit Embedded Brain Academy](http://fastbitlab.com/). Contains source coude for all exercises done during course, tested and adjusted for [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) board.


## Usage

For this assignments [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board was used, but you can use code and adjust it according to your board.

Src folder contains tests, which covers every major implementation (GPIO, I2C, SPI, USART, IRQs)
The folder structure is described as follows:

|                |Files                            |Description                  |
|----------------|---------------------------------|-----------------------------|
|**/src**	 |`".c"` files		   |Driver headers application layer|
|**/drivers**    |`".h"` and `".c"` files  	   |It contains the hearders and sources files for each peripheral |
|**/drivers/inc**|`"stm32f401xx.h"` <br> `"stm32f401xx_gpio_driver.h"` <br> `"stm32f401xx_i2c_driver.h"` <br> `"stm32f401xx_spi_driver.h"` <br> `"stm32f401xx_spi_driver.h"`   |-- Driver header layer application |
|**/drivers/src**|`".c"` files		   |-- Driver source layer application |

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
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//           Implementation of printf like feature using ARM Cortex M3/M4/ ITM functionality
//           This function will not work for ARM Cortex M0/M0+
//           If you are using Cortex M0, then you can use semihosting feature of openOCD
/////////////////////////////////////////////////////////////////////////////////////////////////////////


//Debug Exception and Monitor Control Register base address
#define DEMCR                   *((volatile uint32_t*) 0xE000EDFCU )

/* ITM register addresses */
#define ITM_STIMULUS_PORT0   	*((volatile uint32_t*) 0xE0000000 )
#define ITM_TRACE_EN          	*((volatile uint32_t*) 0xE0000E00 )

void ITM_SendChar(uint8_t ch)
{

	//Enable TRCENA
	DEMCR |= ( 1 << 24);

	//enable stimulus port 0
	ITM_TRACE_EN |= ( 1 << 0);

	// read FIFO status in bit [0]:
	while(!(ITM_STIMULUS_PORT0 & 1));

	//Write to ITM stimulus port0
	ITM_STIMULUS_PORT0 = ch;
}
```


&nbsp;

After that find function *_write* and replace `__io_putchar(*ptr++)` with `ITM_SendChar(*ptr++)` like in code snippet below
```c
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
	int DataIdx;

	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		//__io_putchar(*ptr++);
		ITM_SendChar(*ptr++);
	}
	return len;
}
```

&nbsp;

&nbsp;
## Contributing 

Pull requests are welcome. If you discover any bug/issue feel free to report it.

## License
[MIT](https://github.com/mattsousaa/STM32F4xx_drivers/blob/master/LICENSE)