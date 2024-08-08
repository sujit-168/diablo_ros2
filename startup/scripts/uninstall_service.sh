#!/bin/bash
 
# Author: sujit-168 su2054552689@gmail.com
# Date: 2023-11-14 19:11:36
# Description: diablo startup
# Copyright (c) 2023 by Tianbot. All Rights Reserved. 


/usr/sbin/diablo_stop
sudo systemctl disable diablo_startup.service
 
sudo rm /usr/sbin/diablo_start
sudo rm /usr/sbin/diablo_stop
sudo rm /usr/sbin/diablo_restart
 
sudo rm /lib/systemd/system/diablo_startup.service
 
exit 0