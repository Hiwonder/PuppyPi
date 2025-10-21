#!/usr/bin/python3
#coding=utf8
#第8章 ROS机器狗拓展课程\2.树莓派扩展板课程\第5课 姿态控制(8.ROS Robot Expanded Course\2.Raspberry Pi Expansion Board Course\Lesson 5 Posture Control)
import os
import sys
import rospy
from ros_robot_controller.msg import RGBState, RGBsState
from std_msgs.msg import *
from sensor_msgs.msg import Imu

# 姿态检测(posture detection)

print('''
**********************************************************
******************功能:姿态检测例程(function: posture detection routine)*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')


# 设置RGB彩灯显示(set the display of RGB color light)
def set_rgb_show(r1,g1,b1,r2,g2,b2):
    led1 = RGBState()
    led1.r = r1
    led1.g = g1
    led1.b = b1
    led1.id = 1

    led2 = RGBState()
    led2.r = r2
    led2.g = g2
    led2.b = b2
    led2.id = 2
    msg = RGBsState()
    msg.data = [led1,led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

rgb_color = 'None'
def run(msg):
    global rgb_color
    
    x = msg.linear_acceleration.x
    y = msg.linear_acceleration.y
   
    if x >= 3: # 机器人左倾斜(the robot tilts to the left)
        if rgb_color != 'red_r':
            print('机器人左倾斜')
            rgb_color = 'red_r'
            set_rgb_show(255,0,0,0,0,0)
        
    elif x <= -3: # 机器人右倾斜(the robot tilts to the right)
        if rgb_color != 'red_l':
            print('机器人右倾斜')
            rgb_color = 'red_l'
            set_rgb_show(0,0,0,255,0,0)
    
    elif y >= 3: # 机器人前倾斜(the robot leans forward)
        if rgb_color != 'red':
            print('机器人前倾斜')
            rgb_color = 'red'
            set_rgb_show(255,0,0,255,0,0)
            
    
    elif y <= -3: # 机器人后倾斜(the robot leans backward)
        if rgb_color != 'blue':
            print('机器人后倾斜')
            rgb_color = 'blue'
            set_rgb_show(0,0,255,0,0,255)
    
    else:
        if rgb_color != 'None':
            print('机器人姿态正常')
            rgb_color = 'None'
            set_rgb_show(0,0,0,0,0,0)

# 关闭检测函数(close detection function)
run_st = True
def Stop():
    global run_st
    run_st = False
    set_rgb_show(0,0,0,0,0,0)
    print('关闭中...')

if __name__ == '__main__':
    # 初始化节点(initialization node)
    rospy.init_node('posture_detect_demo')
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    imu_sub = rospy.Subscriber('/ros_robot_controller/imu_raw', Imu, run)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        
    
