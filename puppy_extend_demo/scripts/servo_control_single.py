#!/usr/bin/env python3
# coding=utf8
# 第8章 ROS机器狗拓展课程\3.空心杯舵机控制课程\第2课 控制空心杯舵机转动(8.ROS Robot Expanded Course\3.Hollow Cup Servo Control Course\Lesson 2 Control Hollow Cup Servo Rotation)
import sys
import time
import signal

sys.path.append('/home/ubuntu/software/puppypi_control')
from ros_robot_controller_sdk import Board
from pwm_servo_control import PWMServoControl

# 控制单个舵机(control single servo)

print('''
**********************************************************
*****************功能:单个舵机控制例程(function: single servo control routine)***********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

# 关闭检测函数(close detection function)
run_st = True
def Stop(signum, frame):
    global run_st
    run_st = False
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    board = Board()
    servo = PWMServoControl(board)
    
    while run_st:
         servo.setPulse(1,1000,2000) # 驱动1号舵机(drive servo No.1)
         time.sleep(2) # 延时(delay)
         servo.setPulse(1,2000,2000) # 驱动1号舵机(drive servo No.1)
         time.sleep(2) # 延时(delay)
    
    servo.setPulse(1,1500,500)
    time.sleep(0.5) # 延时(delay)
