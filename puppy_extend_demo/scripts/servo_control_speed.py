#!/usr/bin/env python3
# coding=utf8
# 第8章 ROS机器狗拓展课程\3.空心杯舵机控制课程\第3课 控制舵机速度(8.ROS Robot Expanded Course\3.Hollow Cup Servo Control Course\Lesson 3 Control Servo Speed)
import sys
import time
import signal
sys.path.append('/home/ubuntu/software/puppypi_control')
from ros_robot_controller_sdk import Board
from pwm_servo_control import PWMServoControl

# 控制舵机速度(control servo speed)

print('''
**********************************************************
*****************功能:舵机速度控制例程(function: servo speed control routine)***********************
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
        servo.setPulse(1,1500,1000) # 驱动1号舵机(drive servo No.1)
        time.sleep(1) # 延时(delay)
        servo.setPulse(1,2500,1000) # 驱动1号舵机(drive servo No.1)
        time.sleep(2) # 延时(delay)
        servo.setPulse(1,1500,1000) # 驱动1号舵机(drive servo No.1)
        time.sleep(2) # 延时(delay)
        servo.setPulse(1,500,500) # 驱动1号舵机(drive servo No.1)
        time.sleep(1) # 延时(delay)
    
    servo.setPulse(1,1500,2000)
    
