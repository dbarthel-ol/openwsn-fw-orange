# Installing a working environment for OpenWSN on OSX 10.10 (Yosemite)

| *document* | Mac OSX 10.10 kickstart for OpenWSN on IoT-LabM3             |
|:----------:|:-------------------------------------------------------------|
| *version*  |  1.0                                                         |
| *date*     |  November, 14th 2014                                         |
| *author*   |  Quentin Lampin, <quentin.lampin@orange.com>                 |


## Table of Contents

|               Sections                                                 |
|:----------------------------------------------------------------------:|
| [Foreword](#foreword)                                                  |
| [Installing HomeBrew](#installing-homebrew)                            |
| [Installing relevant packages](#installing-relevant-packages)          |
| [Installing relevant packages](#installing-relevant-packages)          |
| [Installing TunTap](#installing-tuntap)                                |
| [Installing the toolchain](#installing-the-toolchain)                  |
| [Compiling a binary](#compiling-a-binary)                              |


## Foreword
Setting up a working environment on OSX based on gnu/linux tools gets a little tricky at times (non case-sensitive filesystem, Apple custom baked libraries and programs, 
you name it!). Despair not! 
*Note*: This tutorial is actually notes I've taken when installing my own on november 14th 2014.

## Installing HomeBrew
Open a terminal window and type or copy/paste the following line:
```
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
```
## Installing relevant packages
```
brew install homebrew/versions/gcc49
brew install scons
brew install pip
sudo pip install bottle
sudo pip install PyDispatcher
sudo pip install pyserial

```

## Installing TunTap
install tuntap from [here](http://sourceforge.net/projects/tuntaposx/files/tuntap/20141104/tuntap_20141104.tar.gz/download).

*Note*: Yosemite enforces a no non-signed kext (kernel extensions) policy which forces developers to sign kext binaries. This causes issues
with kexts shipped by package managers such as homebrew, macports and fink.



## Installing the toolchain
grab the arm-gcc compiler for ARM-cortex-M processors from [here](https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q3-update/+download/gcc-arm-none-eabi-4_8-2014q3-20140805-mac.tar.bz2) or on their main [page](https://launchpad.net/gcc-arm-embedded) for newer versions.
Extract it somewhere you know you won't move/delete in the near future and update your $PATH environment to include the bin directory of the toolchain.
```
echo "export PATH=${PATH}:${install_dir}/gcc-arm-none-eabi-eabi-4_8-2014q3/bin" >> ~/{.bashrc,.zshrc}
source ~/{.bashrc,.zshrc}
```

## Compiling a binary
The board/toolchain arguments are : `board=iot-lab_M3` and `toolchain=armgcc`.

Example for the oos_openwsn binary:
```
scons board=iot-lab_M3 toolchain=armgcc oos_openwsn
```

### Troubleshooting
If scons complains about not finding the executable `arm-none-eabi-gcc`, make sure that your `PATH` variable includes the bin directory of your toolchain.
You can display your PATH in the terminal:
```
echo $PATH
```
If your `PATH` does not include it, make sure you've update your `PATH` in `~/.bashrc` or in `~/.zshrc` if you're using zsh. Also, make sure you've sourced
those files if you're still using the same terminal session.


## Flashing a firmware to an IoT-Lab M3 board

Starting with OSX 10.9 (Mavericks), Apple ships his own FTDI kernel extension which seems to pose issues with several
FTDI chips. It is advised to use the kernel extension provided by FTDI [here](http://www.ftdichip.com/Drivers/VCP.htm).
If you have decided to use FTDI's own driver then you need to disable the one shipped by Apple:
```
OLD=`pwd`
cd /System/Library/Extensions/IOUSBFamily.kext/Contents/PlugIns
sudo mv AppleUSBFTDI.kext AppleUSBFTDI.disabled
cd $OLD
```

Yosemite does not allow a userspace program to claim devices that are already attached to a kernel extension.
You first need to unload the FTDI kernel extension, flash the binary and reload the FTDI kernel extension afterwards.

```
sudo kextunload -b com.FTDI.driver.FTDIUSBSerialDriver
<flash command>
sudo kextload -b com.FTDI.driver.FTDIUSBSerialDriver
```
## Using GDB with openocd
iuse the debug.sh script (located in #openwsn-fw-directory/bsp/boards/iot-lab_M3/tools) then start arm-none-eabi-gdb
```
bsp/boards/iot-lab_M3/tools/debug.sh
arm-none-eabi-gdb
```

in gdb: connect to the OpenOCD gdb port, reset and halt the target program, load the program
```
target remote localhost:3123
monitor reset halt
file bin/#firware-binary-on-the-node
```
set your breakpoints and start the program:

```
break openserial.c:47
continue
```

