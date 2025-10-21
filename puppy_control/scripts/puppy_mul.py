#!/usr/bin/python3
# coding=utf8
# Author:Summer
# Email:997950600@qq.com

import os, sys, math
import numpy as np
import rospy
from std_msgs.msg import UInt8, UInt16, Float32, Float64, Bool, String,Float32MultiArray
from std_srvs.srv import Empty, SetBool
# from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32, Polygon
from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName
from geometry_msgs.msg import Twist

ROS_NODE_NAME = 'puppy_control'

sys.path.append('/home/ubuntu/software/puppypi_control')
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup


class PUPPY():
    def __init__(self):
        rospy.init_node("puppy_run_act", anonymous=True)
        rospy.Subscriber('/multi_robot/runActionGroup', String, self.runActionGroupFun)

    def runActionGroupFun(self, msg):
        rospy.logdebug(msg)
        print(msg)
        runActionGroup(msg.data, False)
        return [True, msg.data]


if __name__ == '__main__':
    puppy = PUPPY()

    try:
        rospy.spin()
    except :
        pass
    finally:
        pass
