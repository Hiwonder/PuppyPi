#!/usr/bin/python3
#coding=utf8
#第8章 ROS机器狗拓展课程\2.树莓派扩展板课程\第4课 按键控制(8. ROS Robot Expanded Course\2. Raspberry Pi Expanded Course\Lesson 4 Button Control)
import os
import sys
import rospy
import gpiod
from ros_robot_controller.msg import RGBState,RGBsState,BuzzerState
from std_msgs.msg import *


# 按键控制(button control)

print('''
**********************************************************
*********************功能:按键控制例程(function: button control routine)**********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to turn off the program, please try multiple times if fail)
----------------------------------------------------------
''')

# 关闭RGB彩灯(turn off RGB color light)
def turn_off_rgb():
    led1 = RGBState()
    led1.id = 1
    led1.r = 0
    led1.g = 0
    led1.b = 0
    
    led2 = RGBState()
    led2.id = 2
    led2.r = 0
    led2.g = 0
    led2.b = 0

    msg = RGBsState()
    msg.data = [led1,led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

# 设置RGB彩灯显示(set the display of RGB color light)
def set_rgb_show(r,g,b):

    led1 = RGBState()
    led1.id = 1
    led1.r = r
    led1.g = g
    led1.b = b
    
    led2 = RGBState()
    led2.id = 2
    led2.r = r
    led2.g = g
    led2.b = b

    msg = RGBsState()
    msg.data = [led1,led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

# 关闭检测函数(turn off detection function)
run_st = True
def Stop():
    global run_st
    run_st = False
    turn_off_rgb()
    print('关闭中...')


if __name__ == '__main__':
    # 按键初始化配置(key initialization configuration)
    key1_pin = 25
    key2_pin = 23
    
    chip = gpiod.chip("gpiochip4")
    
    key1 = chip.get_line(key1_pin)
    config = gpiod.line_request()
    config.consumer = "key1"
    config.request_type = gpiod.line_request.DIRECTION_INPUT
    config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
    key1.request(config)

    key2 = chip.get_line(key2_pin)
    config = gpiod.line_request()
    config.consumer = "key2"
    config.request_type = gpiod.line_request.DIRECTION_INPUT
    config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
    key2.request(config)
    
    # 初始化节点(initialization node)
    rospy.init_node('button_control_demo')
    rospy.on_shutdown(Stop)
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    buzzer_pub = rospy.Publisher("/ros_robot_controller/set_buzzer",BuzzerState, queue_size=1)
    rospy.sleep(0.5) # 延时一会(delay for a moment)
    
    buzzer_msg = BuzzerState()

    button_press = False # 按键按压状态(button press status)
    while run_st:
        if key1.get_value() == 0: # 检测到按键1按下(key 1 pressed detected)
            rospy.sleep(0.05) # 延时消抖再检测(delay for debounce before detection)
            if key1.get_value() == 0:
                if not button_press:
                    button_press = True
                    r,g,b = 0,255,0
                    set_rgb_show(r,g,b) #绿色(green)
                    buzzer_msg.freq = 1900
                    buzzer_msg.on_time = 0.5
                    buzzer_msg.off_time = 0.5
                    buzzer_msg.repeat = 1
                    buzzer_pub.publish(buzzer_msg) # 蜂鸣器响0.5秒(buzzer sound for 0.5 seconds)
        
        if key2.get_value() == 0: # 检测到按键2按下(press key2 when detected)
            rospy.sleep(0.05) # 延时消抖再检测(delay for debounce before detecting again)
            if key2.get_value() == 0:
                if not button_press:
                    button_press = True
                    r,g,b = 0,0,255
                    set_rgb_show(r,g,b) #蓝色(blue)
                    buzzer_msg.freq = 1900
                    buzzer_msg.on_time = 0.5
                    buzzer_msg.off_time = 0.5
                    buzzer_msg.repeat = 1
                    buzzer_pub.publish(buzzer_msg) # 蜂鸣器响0.5秒(buzzer emit for 0.5 seconds)
        else:
            if button_press:
                button_press = False
            rospy.sleep(0.05)
        
    
