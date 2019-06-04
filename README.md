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

### IMU Calibration
Arguably one of the most overlooked parts of working with sensors is calibration. To read accelerometer and gyroscope data, I simply wrote to the registers containing these their information. The sensor data is stored in a high and low register, with the low register setting negative data values and the high register setting positive data values. To calculate the actual value of each sensor, I simply shifted the high register by one byte and OR'd it with the low register to obtain the final 16 bit value. Then, I simply adjusted the sensitivity of the accelerometer and gryoscope with the proper scaling value from the datasheet.

`acc[0] = (int16_t)((int16_t)raw[0]<<8|raw[1]);  
acc[1] = (int16_t)((int16_t)raw[2]<<8|raw[3]);  
acc[2] = (int16_t)((int16_t)raw[4]<<8|raw[5]);`
 
Calibrating the magnetometer was slightly more involved given that it's an external sensor separate from the accelerometer and gyroscope. To enable direct ability to read from the magnetometer, I had to first write to the bypass enable register. This gave me the ability to write directly to the magnetometer to read data, rather from reading from the FIFO register.

`data[0] = MPU_INT_BYPASS_CFG;  
data[1] = 0x22;  
i2c_transfer7(I2C, MPU_ADDR, data, 2, data, 0);`
 
The next step in initializing the magnetometer involved retrieving factory calibration values from the correct registers and using those values to convert raw magnetometer readings to microteslas. The conversion formula I used is shown below. 

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/conversion.png "Conversion Formula")

The last part of calibrating the magnetometer is adjusting for hard and soft iron biases. Hard iron biases are simply to adjust for, as they're a result of constant magnetic interferences. To calculate the adjustment for hard iron biases, simply log data in the xyz axes while moving the sensor in a figure eight pattern. When you plot magetometer data in the xyz axes against each other, the result should look roughly look a circle centered around the origin. If it is not centered around the origin, simply take find the average of the maximum x,y, and z values and substract them from the xyz axis readings. This effectively cancels the effects of an hard iron biases. The matlab plot below shows how offset the values for my magetometer were.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/mag.png "Mag Calibration")

### Sensor Fusion Implementation

#### Madgwick Quarternion Representation

The original source of this algorithm is derived the paper listed here at http://x-io.co.uk/res/doc/madgwick_internal_report.pdf 
Below, I will describe the basic mathematical derivation behind using the 9 axis accelerometer to determine a unique representation of the sensor payload. The advantage of this algorithm over conventional Kalman Filters is that is it computationally efficient, involving 270 scalar operations and 5 square root operations per filter operation.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/block.png "Block Diagram")

When you represent an objects trajectory and heading with Euler angles, the euler angle representation is not unique, since any rotation about the axis parallel to the true orientation is equivalent. Quarternion angles are a way of uniquely determining the objects orientation, and they provide several useful properties. Put simply, Quarternions are formed by four components in the form of `a + bi + cj + dk`, where a, b, c, and d are real numbers and i, j, and k are fundamental quarternion units. Listed below are several properties of quarternions. Additional basic information can be found here: https://en.wikipedia.org/wiki/Quaternion

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/quartprop1.png "Quarternion Property 1")
![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/quartprop2.png "Quarternion Property 2")



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
