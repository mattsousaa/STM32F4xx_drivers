## STM32F4xx_drivers

Drivers developed during course [MCU1](https://www.udemy.com/share/101rCkBkcacF1XR3o=/) by [FastBit Embedded Brain Academy](http://fastbitlab.com/). Contains source coude and drivers for all exercises done during the course. All of these were tested and adjusted for [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) board.


## Usage

For this assignments [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) board was used, but you can use the code and adjust it according to your board.

The src folder that contains the test files (which covers every major implementation like GPIO, I2C, SPI, USART and IRQs) is the Application Layer.
The folder structure is described as follows:

📦Src
 ┣ 📜001led_toggle.c \
 ┣ 📜002led_button.c \
 ┣ 📜003HSI_Measurement.c
 ┣ 📜004Button_interrupt.c
 ┣ 📜005Spi_Tx.c
 ┣ 📜006spi_tx_only_avr.c
 ┣ 📜007spi_cmd_handling.c
 ┣ 📜008Spi_IRQ_avr.c
 ┣ 📜009I2C_master_tx_testing.c
 ┣ 📜010i2c_master_rx_testing.c
 ┣ 📜011i2c_master_tx_testing_irq.c
 ┣ 📜012i2c_slave_tx_string.c
 ┣ 📜013i2c_slave_tx_string2.c
 ┣ 📜014uart_tx_rx.c
 ┗ 📜015uart_tx_rx_irq.c

📦drivers
 ┣ 📂Inc
 ┃ ┣ 📜stm32f401xx.h
 ┃ ┣ 📜stm32f401xx_gpio_driver.h
 ┃ ┣ 📜stm32f401xx_i2c_driver.h
 ┃ ┣ 📜stm32f401xx_rcc_driver.h
 ┃ ┣ 📜stm32f401xx_spi_driver.h
 ┃ ┗ 📜stm32f401xx_usart_driver.h
 ┗ 📂Src
 ┃ ┣ 📜stm32f401xx_gpio_driver.c
 ┃ ┣ 📜stm32f401xx_i2c_driver.c
 ┃ ┣ 📜stm32f401xx_rcc_driver.c
 ┃ ┣ 📜stm32f401xx_spi_driver.c
 ┃ ┗ 📜stm32f401xx_usart_driver.c
 
| Folders        |Files                            |Description                  |
|----------------|---------------------------------|-----------------------------|
|**/src**	 |`"001led_toggle.c"` <br> `"002led_button.c"` <br> `"003HSI_Measurement.c"` <br> `"004Button_interrupt.c"` <br> `"005Spi_Tx.c"` <br> `"006spi_tx_only_avr.c"` <br> `"007spi_cmd_handling.c"` <br> `"008Spi_IRQ_avr.c"` <br> `"009I2C_master_tx_testing.c"` <br> `"010i2c_master_rx_testing.c"` <br> `"011i2c_master_tx_testing_irq.c"` <br> `"012i2c_slave_tx_string.c"`	<br> `"013i2c_slave_tx_string2.c"` <br> `"014uart_tx_rx.c"`	<br> `"015uart_tx_rx_irq.c"`   	   |**-- Led Toggle Application <br> -- Led/Button Application <br> -- Measuring Internal Clock Application <br> -- Interrupt Button Application <br> -- SPI transmission test Application <br> -- SPI Transmission Only Application <br> -- SPI Command Application <br> -- SPI and AVR Application <br> -- I2C master testing Application <br> -- I2C master reception Application <br> -- I2C master IRQ testing Application <br> -- I2C Slave testing Application <br> -- I2C Slave long Byte testing Application <br> -- UART TX_RX testing Application <br> -- UART TX_RX IRQ testing Application**  |
|**/drivers**    |**Include and source files**  	   |**Driver Layer** |
|**/drivers/inc**|`"stm32f401xx.h"` <br> `"stm32f401xx_gpio_driver.h"`<br> `"stm32f401xx_i2c_driver.h"` <br> `"stm32f401xx_spi_driver.h"` <br> `"stm32f401xx_rcc_driver.h"`|**-- MCU Specific Header File <br> -- GPIO Header File <br> -- I2C Header File <br> -- SPI Header File <br> -- RCC Header File** |
|**/drivers/src**|`"stm32f401xx_gpio_driver.c"` <br> `"stm32f401xx_i2c_driver.c"` <br> `"stm32f401xx_spi_driver.c"` <br> `"stm32f401xx_rcc_driver.c"`| **-- GPIO Source File <br> -- I2C Source File <br> -- SPI Source File <br> -- RCC Source File** |


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
  * `Floating-point ABI: Software implementation (-mfloat-abi=soft)`

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

## Setup Installation - Windows 10
* ##### STM32Cube IDE installation (v1.3.0)
* [Download installation for Windows](https://www.st.com/en/development-tools/stm32cubeide.html#get-software) (Windows Installer)
* Download the software and install it following the instructions.

## Contributing 

Pull requests are welcome. If you discover any bug/issue feel free to report it and let me know if you want to.

## License
[MIT](https://github.com/mattsousaa/STM32F4xx_drivers/blob/master/LICENSE)
