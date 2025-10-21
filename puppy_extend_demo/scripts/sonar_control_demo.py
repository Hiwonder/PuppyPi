#!/usr/bin/env python3
# 第8章 ROS机器狗拓展课程\4.传感器开发课程\第1课 发光超声波传感器控制(8.ROS Robot Expanded Course\4.Sensor Development Course\Lesson 1 Glowy Ultrasonic Sensor Control)
import os
import sys
import time
import signal
import sensor.Sonar as Sonar

print('''
**********************************************************
*******************功能:超声波控制例程(function:ultrasonic control routine)**********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


# 关闭检测函数(close detection function)
run_st = True
def Stop(signum, frame):
    global run_st
    run_st = False
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    s = Sonar.Sonar()
    s.setRGBMode(0) # 0:彩灯模块,1:呼吸灯模式(0:color light module, 1:breathing light mode)
    s.setRGB(1, (0, 0, 0)) # 关闭RGB灯(turn off RGB light)
    s.setRGB(0, (0, 0, 0))
    while run_st:
        time.sleep(0.1)
        distance = s.getDistance() # 获得检测的距离(obtain detected distance)
        print('distance: {}(mm)'.format(distance))
        if distance <= 300: # 距离小于300mm(the distance is less than 300mm)
            s.setRGB(1, (255, 0, 0)) # 设为红色(set to red color)
            s.setRGB(0, (255, 0, 0))
            
        elif 300 < distance < 500: 
            s.setRGB(1, (0, 255, 0)) # 设为绿色(set to green color)
            s.setRGB(0, (0, 255, 0))
            
        else:
            s.setRGB(1, (0, 0, 255)) # 设为蓝色(set to blue color)
            s.setRGB(0, (0, 0, 255))
            
    s.setRGB(1, (0, 0, 0))
    s.setRGB(0, (0, 0, 0))
    
