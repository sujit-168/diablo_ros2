#!/bin/bash
 
echo "delete remap the device serial port(ttyUSBX) to  diablo"
echo "sudo rm   /etc/udev/rules.d/diablo.rules"
sudo rm   /etc/udev/rules.d/diablo.rules

echo "Restarting udev"

sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish  delete"