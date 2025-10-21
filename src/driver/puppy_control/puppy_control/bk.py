#!/usr/bin/python3
# coding=utf8
# Author:hiwonder

import os, sys, time
import numpy as np

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse
from puppy_kinematics import HiwonderPuppy, PWMServoParams


puppy = HiwonderPuppy(setServoPulse = setServoPulse, servoParams = PWMServoParams(), dof = '8')

# print(dir(puppy))
# print("Attributes:", dir(puppy._HiwonderPuppy__config))
# print("Attributes:", dir(puppy._HiwonderPuppy__Configuration))
# print("DOF", puppy._HiwonderPuppy__config.dof)
# print("LEG_FB:", puppy._HiwonderPuppy__config.LEG_FB)
# print("LEG_LR:", puppy._HiwonderPuppy__config.LEG_LR)
# print("LEG_L2:", puppy._HiwonderPuppy__config.LEG_L2)
# print("LEG_L1:", puppy._HiwonderPuppy__config.LEG_L1)
# print("ABDUCTION_OFFSET:", puppy._HiwonderPuppy__config.ABDUCTION_OFFSET)
# print("FOOT_RADIUS:", puppy._HiwonderPuppy__config.FOOT_RADIUS)
# print("LEG_L3:", puppy._HiwonderPuppy__config.LEG_L3)
# print("LEG_L4:", puppy._HiwonderPuppy__config.LEG_L4)
# print("LEG_L5:", puppy._HiwonderPuppy__config.LEG_L5)
# print("LEG_L6:", puppy._HiwonderPuppy__config.LEG_L6)
# print("LEG_L7:", puppy._HiwonderPuppy__config.LEG_L7)
# print("LEG_L8:", puppy._HiwonderPuppy__config.LEG_L8)
# print("ANGLE_DAE_ANGLE:", puppy._HiwonderPuppy__config.ANGLE_DAE_ANGLE)
# print("ANGLE_DOF_ANGLE:", puppy._HiwonderPuppy__config.ANGLE_DOF_ANGLE)
# print("ANGLE_GDO_ANGLE:", puppy._HiwonderPuppy__config.ANGLE_GDO_ANGLE)
# for i in dir(puppy._HiwonderPuppy__config):
	# attr = getattr(puppy._HiwonderPuppy__config, i)
	# if not callable(attr) and not i.startswith("__"):
            # print(f"Variable: {i}, Value: {attr}")

for i in dir(puppy._HiwonderPuppy__controller):
    attr = getattr(puppy._HiwonderPuppy__controller, i)
    if not callable(attr) and not i.startswith("__"):
            print(f"Variable: {i}, Value: {attr}")

# for i in dir(puppy.state):
	# attr = getattr(puppy.state, i)
	# if not callable(attr) and not i.startswith("__"):
            # print(f"Variable: {i}, Value: {attr}")

# for i in dir(puppy.command):
	# attr = getattr(puppy.command, i)
	# if not callable(attr) and not i.startswith("__"):
            # print(f"Variable: {i}, Value: {attr}")
