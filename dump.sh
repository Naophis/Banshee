# espcoredump.py  --chip esp32s3 -p /dev/ttyUSB0 info_corefile ./build/main.elf

echo thread apply all bt full
port=`ls /dev/ttyUSB* 2>/dev/null`
if [ -z "$port" ]; then
    port=`ls /dev/ttyACM* 2>/dev/null`
fi
echo ${port}
espcoredump.py --chip esp32s3 -p ${port} dbg_corefile build/main.elf
# espcoredump.py --chip esp32s3 -p /dev/ttyUSB1 info_corefile -t b64  ./build/main.elf
