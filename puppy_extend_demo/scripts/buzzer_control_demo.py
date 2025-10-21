#!/usr/bin/python3
#coding=utf8
#第8章 ROS机器狗拓展课程\2.树莓派扩展板课程\第3课 控制蜂鸣器(8.ROS Robot Expanded Course\2.Raspberry Pi Expanded Course\Lesson 3 Control Buzzer)
import os
import sys
import rospy
from ros_robot_controller.msg import BuzzerState
from std_msgs.msg import *

# 控制蜂鸣器(control buzzer)

print('''
**********************************************************
******************功能:蜂鸣器控制例程(function: buzzer control routine)***********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

if __name__ == '__main__':
    # 初始化节点(initialization node)
    rospy.init_node('buzzer_control_demo')
    
    buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)
    rospy.sleep(0.5) # 延时一会(delay for a second)
    
    buzzer_msg = BuzzerState()

    buzzer_msg.freq = 1900
    buzzer_msg.on_time = 2
    buzzer_msg.off_time = 1
    buzzer_msg.repeat = 1
    buzzer_pub.publish(buzzer_msg) # 蜂鸣器响2秒(buzzer emit for 2 seconds)
    rospy.sleep(3)

    buzzer_msg.freq = 1900
    buzzer_msg.on_time = 0.5
    buzzer_msg.off_time = 0.5
    buzzer_msg.repeat = 1
    buzzer_pub.publish(buzzer_msg) # 蜂鸣器响0.5秒(buzzer emit for 0.5 second)
        
    
