#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/pi/puppy_pi/devel/setup.bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311

python3 /home/pi/puppy_pi/src/puppy_pi_bringup/scripts/camera_disconnect_detect.py
