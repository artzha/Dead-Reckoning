# Description
This repository contains a simple example of a project using the [libopencm3](https://github.com/libopencm3/libopencm3) with th STM32F103x8 microcontroller.

# Dependencies

* arm-none-eabi-gcc
* arm-none-eabi-gdb
* arm-none-eabi-newlib
* openocd (>= 0.8.0)
* make
* gawk

# How to compile and upload the examples

## Preparation
Before compiling, make sure to initialize and update the libopencm3 git submodule:
```
$ git submodule init
$ git submodule update
```

## Compile
```
$ make
```
You can change the default optimization level (-Og) through the OPT flag:
```
$ make clean
$ make OPT=-O3
```

## Upload / Flash:
Before uploading/flashing/debugging for the first time select the debug adapter:
```
$ ./dbgcfg
Please chose the debug interface:
[0] ST-LINK/V2
[1] ST-LINK/V2-1
[2] JLINK
[3] CMSIS-DAP
Choice:
```
After that, your choice will be stored on a local ".interface" file. You can change it at any time by invoking the ```dbgcfg``` script again.

To flash the microcontroller:
```
$ make flash
```

## Clean output files:
To clear only output files from the application code (src directory):
```
$ make clean
```
To clear all output files including the libopencm3:
```
$ make distclean
```

## Generate .bin file:
```
$ make bin
```

## Generate .hex file:
```
$ make hex
```

# Debugging:
You can debug with gdb you can invoke the ```debug``` script (tui mode) or invoke gdb directly:
```
$ arm-none-eabi-gdb *.elf -x gdbcmd
```

If you have [PyCortexMDebug](https://github.com/bnahill/PyCortexMDebug) installed you can invoke gdb with python:
```
$ arm-none-eabi-gdb *.elf -x pygdbcmd
```

This will make inspecting microcontroller's registers and bit fields much more convenient.

# License
The libopencm3 is licensed under the GNU Lesser General Public License version 3. All other files not within the libopencm3 directory are lincesed under the 3-Clause BSD license.
