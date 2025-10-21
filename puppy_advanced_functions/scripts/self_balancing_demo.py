#!/usr/bin/python3
# coding=utf8
# 第7章 ROS机器狗创意课程\5.机身自平衡(7.ROS Robot Creative Course\5.Robot Self-balancing)
import sys
import cv2
import math
import rospy
import threading
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *

from puppy_control.msg import Velocity, Pose, Gait
from sensor_msgs.msg import Imu


ROS_NODE_NAME = 'self_balancing_demo'


Stand = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
PuppyPose = Stand.copy()


class MPU6050():

    def __init__(self):
        rospy.Subscriber('/ros_robot_controller/imu_raw', Imu, self.GetImuFun)
        self.data = {'accel':[0,0,0], 'gyro':[0,0,0]}
        self.SecondOrderFilterX = self._SecondOrderFilter()
        self.SecondOrderFilterY = self._SecondOrderFilter()
        
    def GetImuFun(self,msg):
        self.data['accel'][0] = msg.linear_acceleration.x
        self.data['accel'][1] = msg.linear_acceleration.y
        self.data['accel'][2] = msg.linear_acceleration.z
        self.data['gyro'][0] = msg.angular_velocity.x
        self.data['gyro'][1] = msg.angular_velocity.y
        self.data['gyro'][2] = msg.angular_velocity.z


    def _SecondOrderFilter(self):
        # 二阶滤波算法(the second-order filtering algorithm)
        x1=0
        x2=0
        y1=0
        angle = 0
        K2 =0.02
        def fun( angle_m, gyro_m, dt = 0.01):
            nonlocal x1
            nonlocal x2
            nonlocal y1
            nonlocal angle
            nonlocal K2
            x1=(angle_m-angle)*(1-K2)*(1-K2)
            y1=y1+x1*dt
            x2=y1+2*(1-K2)*(angle_m-angle)+gyro_m
            angle=angle+ x2*dt
            return angle

        return fun

    def get_euler_angle(self, dt = 0.01):
        # data = self.get_all_data()
        data = self.data
        accel_Y = math.atan2(data['accel'][0],data['accel'][2])*180/math.pi
        gyro_Y = data['gyro'][1]
        angleY = self.SecondOrderFilterY(-accel_Y,gyro_Y,dt)

        accel_X = math.atan2(data['accel'][1],data['accel'][2])*180/math.pi
        gyro_X = data['gyro'][0]
        angleX = self.SecondOrderFilterX(accel_X,gyro_X,dt)

        # return {'pitch':0, 'roll':0, 'yaw':0}
        return {'pitch':-math.radians(angleX), 'roll':-math.radians(angleY), 'yaw':0}



if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    mpu = MPU6050()
    rospy.sleep(0.2)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    
    debug = False

    rate = rospy.Rate(100)#100Hz 0.01s
    try:
        while not rospy.is_shutdown():
            angle = mpu.get_euler_angle()
            print(angle) # 弧度(radians)
            if debug == False:
                PuppyPose['pitch'] = -angle['pitch']*1.8
                PuppyPose['roll'] = angle['roll']*1.8
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
            rate.sleep()

    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
