#!/bin/bash
 
# Author: sujit-168 su2054552689@gmail.com
# Date: 2023-11-14 19:11:36
# Description: diablo startup
# Copyright (c) 2023 by Tianbot. All Rights Reserved. 

# enchance authority
# chmod +x diablo_start diablo_stop diablo_restart

# config diablo_startup.service
sudo cp diablo_start /usr/sbin/
sudo cp diablo_stop /usr/sbin/
sudo cp diablo_restart /usr/sbin/
 
sudo cp diablo_startup.service /lib/systemd/system/
 
sudo systemctl enable diablo_startup.service
 
exit 0