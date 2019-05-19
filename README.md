# Description

## Introduction

This repository is a collection of my code projects as I learn the aspects of programming an STM32F103C8 Cortex M3 micro controller at the register level. The primary code project in this repository implements a dead-reckoning tool to determine the user's position by using a 9 axis accelerometer and Kalman Filters. This computation will be run in parallel by multiple STM32's and accelerometers to create an additional layer of redundancy and discrepancies will be resolved using the Byzantine General's Algorithm.

Note: I decided to develop this project simply using predefined CMSIS registers to gain a better understanding of how to develop high efficiency and robust code for microproccesors.

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
