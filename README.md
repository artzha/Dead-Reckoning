# Position Tracking Algorithms and Synchronized Computations In Embedded Systems

## Introduction

This repository is a collection of my code projects as I learn the aspects of programming an STM32F103C8 micro controller with minimal library abstractions. The primary code project in this repository implements a dead-reckoning tool to determine the user's position by using a 9 axis accelerometer. Multiple sensor fusion and noise adjustment algorithms were tested for accuracy, such as using Madgwick's Quarternion and Kalman Filtering algorithms. In order to add an additional layer of redundancy for mission critical applications, multiple STM32's and accelerometers will run the identical computation synchronously and descrepancies will be resolved using the Byzantine General's Algorithm.

## Project Overview
1. I2C Communication
2. IMU Calibration
3. Sensor Fusion Implementation

### I2C Communication

#### Setup
I decided to use this as an opportunity to understand and implement I2C communications. The basic features of I2C remain the same with this microcontroller. To start, I the RCC registers to set the clock frequency to 16 MHz.
`RCC->CFGR       |= RCC_CFGR_PLLSRC;     // set PLL source to HSE
 RCC->CFGR       |= RCC_CFGR_PLLMULL2;   // multiply by 2`
The second part of the setup process is to intialize gpio pins 6 and 7 on port B to use open drain and set the pins' clock speed. 
`GPIOB->CRL  |= GPIO_CRL_MODE6_1; // sets alternate function open drain mode
 GPIOB->CRL  |= GPIO_CRL_MODE7_1; // sets alternate function open drain mode
 I2C1->CR2 &= ((uint16_t)0xFFC0); // Clear FREQ [5:0] bits
 I2C1->CR2 |= ClockSpeed; // Ex: 0x10 sets to f_pclk to 16 MHz and is equal to 16`
The last part of the setup involves setting up the rest of the control registers for I2C, renabling the peripherals, and setting the ACK bit.

#### Write and Reading
Reading/writing is tricky to setup on this microcontroller as read/write procedures are different depending on whether it is 1,2, or >= 3 bytes. While the specifics on the differences for these operations are too long to succintly describe below, the jist of it involves setting/resetting start bits and clearing status registers `SR1` and `SR2`. Additional details on this can be found STM32's documentation for I2C optimization. 

#### Debugging
As expected, there were a few hiccups that occurred. The issues were attributed to register specific settings and calculations for the correct clock frequency for I2C. I resolved these issues using a logic analyzer to check the waveforms.
![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/logic-i2c.png "Logic Analyzer")


With a working toolchain, all projects can be built from within their project directory.  The `Makefile` file **REQUIRES** modification in order to set the paths to the build tools.

The following tools are used for these projects:
* [ARM-GCC](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads) compiler toolchain.
* [stm32flash](https://sourceforge.net/projects/stm32flash/) flash tool using the on-board STM32 serial bootloader over UART.
* [st-link](https://github.com/texane/stlink) flash tool using an ST-LINK V2 USB programmer.
* [Official STM32 CMSIS](http://www.st.com/en/embedded-software/stm32cube-mcu-packages.html) files as part of their STM32Cube MCU packages.
* libopencm3 files for a non-optimized implementation of the project with libopencm3 

### Development Hardware

All example projects are using a ["Blue Pill"][blue pill] board as the target hardware platform.

### Common Directories

#### [CMSIS/](CMSIS)

Contains the [Cortex CMSIS](https://developer.arm.com/embedded/cmsis) files needed to build the projects.  These files have been repackaged from the official [STM32 CMSIS](http://www.st.com/en/embedded-software/stm32cube-mcu-packages.html) files.

#### [template/](template)

The template directory contains the files necessary to start a new STM32F103 project.

#### [gdb/](gdb)

The gdb directory contains gbd command files used for debugging and flashing.

[blue pill]: http://wiki.stm32duino.com/index.php?title=Blue_Pill

#### Credits
A large part of the original Makefile initialization was originally developed by getoffmyhack.

# License
The libopencm3 is licensed under the GNU Lesser General Public License version 3. All other files not within the libopencm3 directory are lincesed under the 3-Clause BSD license.
