# espcoredump.py  --chip esp32s3 -p /dev/ttyUSB0 info_corefile ./build/main.elf

echo thread apply all bt full

espcoredump.py --chip esp32s3 -p /dev/ttyUSB0 dbg_corefile build/main.elf
# espcoredump.py --chip esp32s3 -p /dev/ttyUSB1 info_corefile -t b64  ./build/main.elf
