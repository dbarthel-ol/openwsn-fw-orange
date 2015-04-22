#!/bin/bash

BIN_FOLDER=${0%/*}
openocd -f "${BIN_FOLDER}/iotlab-m3-jtag.cfg" \
	-f "target/stm32f1x.cfg" \
	-c "gdb_port 3123" \
  	-c "tcl_port 4123" \
  	-c "telnet_port 5123" \
	-c "init" \
	-c "targets" #\
#	-c "reset halt"

