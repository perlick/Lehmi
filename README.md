# Lemhi Firmware
## Development environment
There are three main components to the development environment/tolchain. 
1. ARM compiler. The gcc compiler should be accessible via the `arm-none-eabi-gcc` command.
2. GDB Client. This is the GDB client interface you will be interacting with. Direclty via the command line or indirectly with vscode or some other GDB UI wrapper
3. GDB Server. In my case OpenOCD. This is the thing that will handle all interaction with the STM32.

## ARM Compiler
I got the official arm toolchain from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads. Then just unzip and add the bin directory the the PATH in your .bashrc. I chose to install in the /opt directory. If you're not able to run the Makefile after installing the comiler, consult the interwebs

## GDB client
This should be installed with the arm toolchain. I'm using the vscode extension 'cortex-debug' by Marus25. to integrate with vscode.

To run the GDB client from the command line, invoke with `arm-none-eabi-gdb` then connect to GDB server `target remote 192.168.0.30:3333`. Then yu can load and run a program by
```
(gdb) file <client side elf file>
(gdb) monitor reset halt
(gdb) monitor flash protect 0 0 15 off
(gdb) monitor stm32f0x mass_erase 0
(gdb) monitor flash write_image /tmp/executable.elf
(gdb) monitor reset halt
```

## GDB Server (OpenOCD)
The last component to this toolchain is openocd. OpenOCD can communicate with the STLink to access the microcontroller over the SWD(?) interface. It can also recieve commands from the GDB clint that tell it to grab certain values etc.

The GDB server needs to be running before you can connect via the client and do anything else. To run the OpenOCD, use this command. `openocd -f /usr/local/share/openocd/scripts/interface/stlink.cfg -f /usr/local/share/openocd/scripts/target/stm32f0x.cfg -f /usr/local/share/openocd/scripts/test.cfg -c "init"` My test.cfg file contains 'bindto 0.0.0.0'. Not sure why that is. I think it let's me access the GDB port from another machine. 

## My setup
I use a raspberry pi as a "host PC" for my STM32. So the rpi has the STLink connected via USB and hardware attached to that. The RPI also runs OpenOCD. Then VSCode, the ARM compiler, and GDB Client all run on my laptop. 
The 'beggining' of my workflow is to run `make` in my vscode terminal. Then F5 will call start cortex-debug with .vscode/launch.json. I've customized this configuration to copy the elf file over ssh to the rpi, then call the correct GDB commands to flash the STM32. 