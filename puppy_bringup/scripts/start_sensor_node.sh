#!/bin/bash
source /home/pi/.bashrc
source /opt/ros/noetic/setup.bash
source /home/pi/puppy_pi/devel/setup.bash
# export ROS_HOSTNAME=192.168.149.1
# export ROS_MASTER_URI=http://192.168.149.1:11311

roslaunch /home/pi/puppy_pi/src/puppy_pi_bringup/launch/start_sensor.launch
