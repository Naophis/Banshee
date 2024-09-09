#!/bin/bash

binary=`find ./build -maxdepth 1 | grep bin$ `
partition=`find ./build/partition_table -maxdepth 1 | grep bin$ `
bootloader=`find ./build/bootloader -maxdepth 1 | grep bin$ `
echo ${binary}
echo ${partition}
echo ${bootloader}
port=`ls /dev/ttyUSB* 2>/dev/null`
if [ -z "$port" ]; then
    port=`ls /dev/ttyACM* 2>/dev/null`
fi
echo ${port}
esptool.py --chip esp32s3 -p ${port} -b 2000000 --before=default_reset --after=hard_reset write_flash --flash_mode dio \
--flash_freq 80m --flash_size 4MB \
0x0 ${bootloader} \
0x8000 ${partition} \
0x10000 ${binary}