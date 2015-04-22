#!/bin/bash

BIN_FOLDER=${0%/*}
openocd -f "${BIN_FOLDER}/iotlab-m3-jtag.cfg" \
	-f "target/stm32f1x.cfg" \
	-c "init" \
	-c "targets" \
	-c "reset halt" \
	-c "reset init" \
	-c "flash write_image erase $1" \
	-c "verify_image $1" \
	-c "reset halt" \
	-c "reset init" \
	-c "reset run"\
	-c "shutdown"
