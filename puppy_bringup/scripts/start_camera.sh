#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/pi/puppypi/devel/setup.zsh

roslaunch /home/pi/puppypi/src/puppy_bringup/launch/usb_cam.launch
