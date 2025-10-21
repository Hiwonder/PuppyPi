#!/usr/bin/python3
#coding=utf8
import os
import sys
import time
import math
import rospy
import gpiod

from std_msgs.msg import *
from puppy_control.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************功能:触摸控制例程(function: touch control routine)*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

# 触摸模块接扩展板上的IO22、IO24接口(connect the touch module to the IO22 and IO24 interfaces on expansion board)

PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
# stance_x：4条腿在x轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the X-axis, measured in centimeters)
# stance_y：4条腿在y轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the Y-axis, measured in centimeters)
# x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm(the distance traveled by the four legs along the x-axis determines the degree of forward or backward tilt during walking: smaller distances lead to more forward tilt, while larger distances result in more backward tilt. Adjusting the x_shift parameter can help maintain balance during the dog's movement, measured in centimeters)
# height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm(the height of the dog, measured from the toe to the axis  of rotation of the thigh, is in centimeters)
# pitch： 狗身体的俯仰角，单位弧度(the pitch angle of the dog's body, measured in radians)


GaitConfig = {'overlap_time':0.2, 'swing_time':0.2, 'clearance_time':0.0, 'z_clearance':3}
# overlap_time:4脚全部着地的时间，单位秒(the time when all four legs touch the ground, measured in seconds)
# swing_time：2脚离地时间，单位秒(the time duration when legs are off the ground, measured in second)
# clearance_time：前后脚间隔时间，单位秒(the time interval between the front and rear legs, measured in seconds)
# z_clearance：走路时，脚抬高的距离，单位cm(the distance the paw needs to be raised during walking, measured in centimeters)


# 触摸模块接扩展板上的IO22、IO24接口(connect the touch module to the IO22 and IO24 interfaces on the expansion board)
touch_pin = 22
chip = gpiod.chip("gpiochip0")
    
touch = chip.get_line(touch_pin)
config = gpiod.line_request()
config.consumer = "touch"
config.request_type = gpiod.line_request.DIRECTION_INPUT
config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
touch.request(config)

# 关闭检测函数(close detection function)
run_st = True
def Stop():
    global run_st
    run_st = False
    print('关闭中...')


if __name__ == '__main__':
    # 初始化节点(initialization node)
    rospy.init_node('touch_control_demo')
    rospy.on_shutdown(Stop)
    
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    rospy.sleep(0.5) # 延时一会(delay for a moment)
    # 机器狗站立(robot dog stands up)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    
    
    click = 0
    squat = True
    while run_st:
        state = touch.get_value()  
        rospy.sleep(0.05)
        if not state:
            detect_time = time.time()+1
            while time.time() < detect_time:
                state = touch.get_value()  
                rospy.sleep(0.1)
                if not state:
                    click += 1
                    rospy.sleep(0.1)
            
            if click == 1:
                click = 0
                if squat:
                    # 机器狗下蹲(the robot dog crouches down)
                    PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-6, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                            height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
                    rospy.sleep(1)
                    squat = False
                    
                elif not squat:
                    # 机器狗站立(the robot dog stands up)
                    PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                            height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
                    rospy.sleep(1)
                    squat = True
                    
            elif click == 2:
                click = 0
                # 机器人抖一抖(the robot shakes a little)
                for i in range(5):
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                        height=PuppyPose['height'], roll=math.radians(3.5), pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 50)
                    rospy.sleep(0.13)
                    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                        height=PuppyPose['height'], roll=math.radians(-3.5), pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 50)
                    rospy.sleep(0.13)
                
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                    height=PuppyPose['height'], roll=math.radians(0), pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 300)
                rospy.sleep(1)
       
        
    
