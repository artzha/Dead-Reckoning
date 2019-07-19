# Position Tracking Algorithms and Synchronized Computations In Embedded Systems

## Introduction

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/FullSytem.jpeg "Full System")

This repository is a collection of my code projects as I learn the aspects of programming an STM32F103C8 micro controller with minimal library abstractions. The primary code project in this repository implements a dead-reckoning tool to determine the user's position by using a 9 axis accelerometer. Multiple sensor fusion and noise adjustment algorithms were tested for accuracy, such as using Madgwick's Quarternion and Kalman Filtering algorithms. In order to add an additional layer of redundancy for mission critical applications, multiple STM32's and accelerometers will run the identical computation synchronously and descrepancies will be resolved using methods inspired by the Byzantine Generals Algorithm and weighted average polling.

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

The original source of this algorithm is derived the paper listed here at http://x-io.co.uk/res/doc/madgwick_internal_report.pdf I will not cover all of the aspects of this paper, but I will attempt to remark about parts that I believe are critical as well as impart an intuitive understanding of what's occuring.
Below, I will describe the basic mathematical derivation behind using the 9 axis accelerometer to determine a unique representation of the sensor payload. The advantage of this algorithm over conventional Kalman Filters is that is it computationally efficient, involving 270 scalar operations and 5 square root operations per filter operation. Later, I will also cover an implementation with Kalman Filters for comparison in both computation intensity and accuracy.

When you represent an objects trajectory and heading with Euler angles, the euler angle representation is not unique, since any rotation about the axis parallel to the true orientation is equivalent. Quarternion angles are a way of uniquely determining the objects orientation, and they provide several useful properties. Put simply, Quarternions are formed by four components in the form of `a + bi + cj + dk`, where a, b, c, and d are real numbers and i, j, and k are fundamental quarternion units. Listed below are several properties of quarternions. Additional basic information can be found here: https://en.wikipedia.org/wiki/Quaternion

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/quartprop1.png "Quarternion Property 1")
![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/quartprop2.png "Quarternion Property 2")

We can denote the quarternion product by the symbol shown below. This allows us to define compound orientations using Hamilton's rule.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/hamilton.png "Hamilton")

Using this rule, we define the bottom formula below to describe the transformation from one orientation representation of vector a in frame A to the same vector b but instead in frame B. A geometricly intuitive way of approaching this formula is by imagining that you're rotating quarternion AB about the vector A. We then rotate the new vector by the conjugate of quarternion AB, which is actually equivalent to rotating the vector A about quaternion BA. This allows us to transform the same vector representation of our object into two different reference frames. This is useful because it gives us a closed form, efficient computational way to not only representing orientations uniquely, but also transform an orientation from one frame to the next. To simply the calculation, the rotation matrix AB can be reduced to the second figure below.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/rotate1.png "Rotation Formula")
![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/rotate2.png "Rotation Matrix")

For more practical purposes in real aerospace or position tracking models, we can also quickly convert this quarternion representation to euler angles. This allows us to take advantage of the useful properties of quarternions while maintaining the intuitive quality euler angles provide us. The conversion formula from quarternions to euler angles are shown below.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/euler.png "Conversion Formula")

With this background, we can now proceed to understanding how we take sensor data and convert them into this representation format. Arguably the most crucial sensor for this calculation is the gyroscope, as it measures the raw angular rate of change. Taking the derivative of the sensor's orientation relative to Earth and rotating this orientation by the angular rate of change, we can calculate the updated angular rate of change of the sensor. Using simple calculus, we take this angular rate of change, multiply by delta t, and add it to the initial sensor orientation to determine what the updated orientation is.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/orientationAngular.png "orientationAngular")
![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/orientationDerivative.png "orientationDerivative")
![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/orientationCalculation.png "orientationCalculation")

So far, we have been assuming that our orientation is already in an unique quarternion form. However, the gyroscope data alone is not enough to allow a complete determination of the orientation. To do so, we need to use the accelerometer and magnetometer to give us the direction of the sensor frame relative to Earth. To compute orientation, we simply take the current orientation estimate, rotate it in the reference direction to align with Earth's reference frame, rotate it once more to acheive the same orientation w.r.t Earth's reference frame instead, and then align it with the measured direction of the sensor. This can be treated as a cost function, in which we optimize to determine the optimum quarternion orientation value, which will be the complete solution we use.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/costfunction.png "costfunction")

To actually minimize this cost function, I simply used gradient descent, as it both computationally efficient and relatively easy to implement. For those unfamiliar with gradient descent, the computation process goes as follows. We take the initial quarternion orientation and subtract it by a step size multiplied bythe normalized gradient of the orientation. We normalize the the gradient because we simply want to determine the change in direction of the orientation, not madnitude. This gradient is by definition the Jacobian matrix times the cost function itself. For those unfamiliar with the Jacobian, in lamens terms, it is simply a scaling factor used when changing between different coordinate systems

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/gradient.png "gradient1")
![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/gradient2.png "gradient2")

One thing to note is the step size. Typically, step size is adjusted for during each step of the process by using the second derivative of the cost function to guage how much it needs to be adjusted. However, because of the computational load of such a task, it is more beneficial for memory/processer limited microcontrollers to instead calculate the step size using the formula shown below. As long as the rate of convergence for the step size is greater than the angular rate of change, we are guaranteed that it will converge. Understanding this, we can update the step size as the physical orientation rate of change of the sensor multiplied by the time period in which it occurs and also alpha, a manually adjusted scaling unit to account to noise in accelerometer and magnetometer measurements.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/stepsize.png "step size")

Moving on, we also have to account for the magnetic distortion as a result of intereferences like electronic devices and metal constructions. Declination errors cannot be corrected for without an additional reference for heading. To account for this error, I will be using the declination at my current location from NOAA's website. More sophisticated implementations of this algorithm can also store a declination data structure to lookup and constantly update the declination as needed. Inclination errors, however, can be compensated by using the accelerometer as an additional measure of attitude. Following the same pattern as earlier, we rotate the magnetometer measurement in the sensor's last measured orientation and remove the effects of erroneous inclication and normalize b_t to only have components in the x and z axes, ensuring that any magnetic interterences simply affect the heading.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/magCalibration.png "Mag Compensation")

Compensating for the gyroscope bias is a similar process, in which you compensate for any gyroscope bias by determining a suitable bias constant and subtracting this factor from the measurement to obtain a compensated value. Putting all of this moving pieces together, we finally construct the algorithm shown below, which displays how the magnetometer measurements/compensation, accelerometer/magnetometer fusion, and gyroscope integration with gradient descent work together to help us continuously update the quarternion representation of our sensor's orientation.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/block.png "Block Diagram")

### Synchronization Algorithm

#### Precision Time Protocol

For this project, I chose to synchronize the clocks for three microcontrollers by implementing a lightweight precision time protocol, also known as PTP. My implementation is accurate within the sub millisecond range, and is easily adjustable to be accurate within the sub microsecond range, making it suitable for control and navigation systems. PTP was originally designed to synchronize networks and systems that required precise timing but lack access to satellite navigation signals. This aligns well with the motivation for dead reckoning position tracking because I too will not rely on any gps or satellite signals for navigation. I based my implementation on a research paper on clock synchronization for networked control systems, which can be found here: https://www.researchgate.net/publication/252228176_Clock_Synchronization_for_Networked_Control_Systems_Using_Low-Cost_Microcontrollers. More details on my implementation can be found below.

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/syncProtocol1.png "Synchronization Diagram")

This algorithm requires minimal processing power and is relatively simple to implement, making it suitable for applications with smaller microcontrollers. The synchronization goes as follows. After initializing peripherals and I2C communication on the master device, it requests the time from its slave controllers. The slave microcontroller handles this request in an I2C interrupt service function to ensure that the response is sent quickly. In the ISR function, the slave first checks that the I2C `SR1_ADDR` bit is matches its own I2C address. If so, the slave saves the current value to complete the read request. After the master receives the slave time, it computes the difference and writes this offset value to the slave. Upon receiving the message, the slave updates its timer value with this offset. This process is repeated once more to account for the time delay on the I2C line. After the master/slave complete their second handshake time exchange, the slave can now correct for the time delay in the time exchange. Using this process, the time latency between the master and slave devices can be minimized. It is important to note that this method assumes that the time delay is symmetrical between master and slaves.

###Triple Redundancy Polling Algorithms

![network structure](https://github.com/KingArthurZ3/Dead-Reckoning/blob/master/rsc/polling.png "Polling Diagram")

Redundant systems are commonly used in critical applications that require high tolerance
for system failures and dependability. While its application are widespread, a few common applications of redundant systems are used in aerospace navigation systems, autonomous transportation, and electrical distribution for power grids. I designed my electrical assembly for use in navigation systems, thus it made sense to implement polling algorithms to ensure high reliability. For this project, I decided to create a simplified version of the Byzantine General's algorithm for general decisions and a weighted average resolution method as backup.

#### Byzantine General Inspiration

Although the Byzantine Generals Algorithm is a popular choice for this application, it would not have worked for my application given that I do not have a way of communicating between the slave devices. Because of this, there is no way of guaranteeing the algorithm's accuracy if there exists more than a single traitor and the master is completely loyal. Therefore, I chose to use the simple majority decision making procedure. Upon receiving orientation calculations from both slaves, the master begins by calculating the average value of all three microcontrollers. To determine each microcontroller's orientation accuracy, the pitch, roll, and yaw values are checked that they are all within 2 standard deviations of the average value. If at least two microcontrollers agree on the same orientation, then the average of the correct values from each device are saved as the most recent orientation determination. This method is suitable for the majority of all discrepancies encountered. Disregarding radiation anomalies, it is unlikely that bit flips will occur on more than one device during each synchronization cycle. However, I used a method of using weighted averages to resolve special cases where two or even all three orientation calculations are vastly different.

#### Weighted Average Percentage Resolution

In the case multiple microcontrollers diverge from each other, the master device stores information of past accuracy for each connected device. More accurately, it stores the number of times each device has stayed within the allowed tolerance and the total number of synchronization processes ran. Oftentimes, the main reason why a device is consistently faulty is due to either software issues or permanently damaged transistors and miscellaneous hardware issues. It could also be due to connection issues between the slave and master. Because of this, it is reasonable to assume that a device that has consistently been faulty will continue to behave in the same way. For this reason, the method of using weighted averages is especially efficient to permanently disregarding information from traitorous devices. Using previous knowledge about each controller's overall accuracy, the algorithm selects the device which holds the highest accuracy as the correct orientation value. However, the selected device's percentage will not be incremented in this situation because we still do not know for certain if this device is truly correct. We can only speculate that this device is more likely to be accurate in comparison to the others. With this, we have now accounted for all discrepancy scenarios.





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
