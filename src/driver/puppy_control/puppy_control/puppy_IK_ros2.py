#!/usr/bin/python3
# coding=utf8
# Author: hiwonder
import os, sys, time
import numpy as np

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse
from puppy_kinematics import HiwonderPuppy, PWMServoParams
puppy = HiwonderPuppy(setServoPulse=None, servoParams=PWMServoParams(), dof='8')

foot_locations = np.array([ [-1,  -1,  -1,   -1], # X
                            [ 0,    0,   0,    0], # Y
                            [-10,   -10,  -10,   -10]  # Z
                            ])


foot_locations = foot_locations / 100

joint_angles = puppy.fourLegsRelativeCoordControl(foot_locations)


print("Servo angles (in degrees):")
print(joint_angles * 57.3)


while True:
    time.sleep(0.001)

