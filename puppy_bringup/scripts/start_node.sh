#!/bin/bash

export DISPLAY=:0
source /opt/ros/noetic/setup.bash
source /home/pi/puppy_pi/devel/setup.bash
# export ROS_HOSTNAME=192.168.149.1
# export ROS_MASTER_URI=http://192.168.149.1:11311

sleep 5
sudo /home/pi/puppy_pi/src/puppy_pi_bringup/scripts/start_sensor_node.sh &
sleep 8
roslaunch /home/pi/puppy_pi/src/puppy_pi_bringup/launch/start_node.launch
