#!/bin/bash

# Author: sujit-168 su2054552689@gmail.com
# Date: 2023-11-14 19:11:36
# Description: diablo startup
# Copyright (c) 2023 by Tianbot. All Rights Reserved. 

# https://www.corvin.cn/707.html


source /opt/ros/galactic/setup.bash
source /home/tianbot/diablo_ros2_ws/install/setup.bash

ros2 launch diablo_ctrl diablo_bringup.launch.py

sleep 3

ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh

sleep 3

ros2 launch diablo_ctrl diablo_bringup.launch.py

exit 0