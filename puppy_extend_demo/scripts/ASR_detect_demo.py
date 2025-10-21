#!/usr/bin/env python3
# coding=utf-8

import os
import time
import rospy
import serial
import binascii
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from ros_robot_controller.msg import RGBState, RGBsState

print('''
**********************************************************
*******************功能:语音识别例程(function: voice recognition routine)************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
    * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close the program, please try multiple times if fail)
----------------------------------------------------------
''')

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
    msg.data = [led1, led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)

def set_rgb_show(r, g, b):
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
    msg.data = [led1, led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.01)


run_st = True
def Stop():
    global run_st
    run_st = False
    turn_off_rgb()
    print("关闭中...")

def parse_serial_data(data):
    hex_data = ' '.join(format(byte, '02X') for byte in data)
    print(f"Received data: {hex_data}")

    if hex_data == "AA 55 00 8A FB":  # 红灯
        set_rgb_show(255, 0, 0)  # 红色
    elif hex_data == "AA 55 00 8B FB":  # 绿灯
        set_rgb_show(0, 255, 0)  # 绿色
    elif hex_data == "AA 55 00 8C FB":  # 蓝灯
        set_rgb_show(0, 0, 255)  # 蓝色
    elif hex_data == "AA 55 00 09 FB":  #停止
        set_rgb_show(0, 0, 0)  
        print("停止识别")
        global run_st
        run_st = False

if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    rospy.init_node('ASR_detect_demo')
    rospy.on_shutdown(Stop)
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
    rospy.sleep(0.2)  

    while run_st:
        if ser.in_waiting > 0:
            data = ser.read(5)  
            parse_serial_data(data)
        rospy.sleep(0.1)
