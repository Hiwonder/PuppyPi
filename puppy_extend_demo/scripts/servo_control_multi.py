#!/usr/bin/env python3
# coding=utf8
import sys
import time
import signal
HomePath = '/home/pi'
sys.path.append(HomePath + '/PuppyPi_PC_Software')
from PWMServoControl import *

# 控制多个舵机(control multiple servos)

print('''
**********************************************************
*****************功能:多个舵机控制例程(function: multiple servo control routine)***********************
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
    servo = PWMServo()
    
    while run_st:
        servo.setPulse(1,1000,2000) # 驱动1号舵机(drive servo No.1)
        servo.setPulse(3,2000,2000) # 驱动3号舵机(drive servo No.3)
        time.sleep(2) # 延时(delay)
        servo.setPulse(1,2000,2000) # 驱动1号舵机(drive servo No.1)
        servo.setPulse(3,1000,2000) # 驱动3号舵机(drive servo No.3)
        time.sleep(2) # 延时(delay)
    
    servo.setPulse(1,1500,2000)
    servo.setPulse(3,1500,2000)
    
