#!/usr/bin/python3
#coding=utf8
# 第8章 ROS机器狗拓展课程\2.树莓派扩展板课程\第2课 控制RGB彩灯(8.ROS Robot Expanded Course\2.Raspberry Pi Expansion Board course\Lesson 2 Control RGB Color Light)
import os
import sys
import rospy
from std_msgs.msg import *
from ros_robot_controller.msg import RGBState, RGBsState


print('''
**********************************************************
****************功能:RGB彩灯控制例程(function: RGB color light control routine)************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

# 关闭RGB彩灯(turn off RGB color light)
def turn_off_rgb():
    led1 = RGBState()
    led1.r = 0
    led1.g = 0
    led1.b = 0
    led1.id = 1

    led2 = RGBState()
    led2.r = 0
    led2.g = 0
    led2.b = 0
    led2.id = 2
    msg = RGBsState()
    msg.data = [led1,led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

# 设置RGB彩灯显示(set the display of RGB color light)
def set_rgb_show(r,g,b):
    led1 = RGBState()
    led1.r = r
    led1.g = g
    led1.b = b
    led1.id = 1

    led2 = RGBState()
    led2.r = r
    led2.g = g
    led2.b = b
    led2.id = 2
    msg = RGBsState()
    msg.data = [led1,led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

# 关闭检测函数(close detection function)
run_st = True
def Stop():
    global run_st
    run_st = False
    turn_off_rgb()
    print('关闭中...')


if __name__ == '__main__':
    # 初始化节点(initialization node)
    rospy.init_node('rgb_control_demo')
    rospy.on_shutdown(Stop)
   
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    rospy.sleep(0.2) # 延时一会，等待订阅生效(delay for a moment for the subscription to take effect)
    
    while run_st:
        r,g,b = 0,0,0
        for r in range(0,255,5): #红色渐亮(the red color gradually brightens)
            set_rgb_show(r,g,b)
            rospy.sleep(0.005)
            
        rospy.sleep(1)
        r,g,b = 0,0,0 
        for g in range(0,255,5): #绿色渐亮(the green color gradually brightens)
            set_rgb_show(r,g,b)
            rospy.sleep(0.005)
        
        rospy.sleep(1)
        r,g,b = 0,0,0 
        for b in range(0,255,5): #蓝色渐亮(the blue color gradually brightens)
            set_rgb_show(r,g,b)
            rospy.sleep(0.005)
        rospy.sleep(1)
        
        
    
