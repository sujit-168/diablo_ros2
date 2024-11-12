#!/bin/bash
 
echo "remap the device serial port(ttyUSBX) to  diablo"
echo "diablo usb connection as /dev/diablo , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy diablo.rules to  /etc/udev/rules.d/"
sudo cp ./diablo.rules  /etc/udev/rules.d

echo "Restarting udev"

sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "