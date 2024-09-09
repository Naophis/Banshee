port=`ls /dev/ttyUSB* 2>/dev/null`
if [ -z "$port" ]; then
    port=`ls /dev/ttyACM* 2>/dev/null`
fi
echo ${port}
esptool.py -p ${port} -b 115200 erase_flash