#!/usr/bin/python3
#coding=utf8
# 第8章 ROS机器狗拓展课程\4.传感器开发课程\第3课 触摸传感器检测(8.ROS Robot Expanded Course\4.Sensor Development Course\Lesson 3 Touch Sensor Detection)
import os
import sys
import math
import rospy
import gpiod
from ros_robot_controller.msg import BuzzerState
from std_msgs.msg import *


print('''
**********************************************************
*******************功能:触摸检测例程(function: touch detection routine)************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')



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
    buzzer_pub.publish(0)

if __name__ == '__main__':
    # 初始化节点(initialization node)
    rospy.init_node('buzzer_control_demo')
    rospy.on_shutdown(Stop)
    
    buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer", BuzzerState, queue_size=1)
    rospy.sleep(0.5) # 延时一会(delay for a moment)
    buzzer_msg = BuzzerState()

    st = 0
    while run_st:
        state = touch.get_value()  #读取引脚数字值(read digital pin value)
        if not state:
            if st :             #这里做一个判断，防止反复响(implement a check here to prevent repeated responses)
                st = 0
                
                buzzer_msg.freq = 1900
                buzzer_msg.on_time = 0.5
                buzzer_msg.off_time = 0.5
                buzzer_msg.repeat = 1
                buzzer_pub.publish(buzzer_msg) # 蜂鸣器响0.5秒(buzzer emits for 0.5 second)
                rospy.sleep(1)
        else:
            st = 1
            
        
    
