#!/usr/bin/env python3
# 第8章 ROS机器狗拓展课程\4.传感器开发课程\第4课 点阵模块显示(8.ROS Robot Expanded Course\4.Sensor Development Course\Lesson 4 Dot-matrix Display)
import os
import sys
import time
from sensor import dot_matrix_sensor

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

print('''
**********************************************************
********************功能:点阵显示实验例程(function: dot-matrix display)********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

# 点阵模块接扩展板上的IO7、IO8接口(connect the dot matrix module to the IO7 and IO8 interfaces on the expansion board)
dms = dot_matrix_sensor.TM1640(dio=7, clk=8)

if __name__ == '__main__':
    # 显示'Hello'(display 'Hello')
    while True:
        try:
            dms.display_buf=(0x7f, 0x08, 0x7f, 0x00, 0x7c, 0x54, 0x5c, 0x00,
                              0x7c, 0x40, 0x00,0x7c, 0x40, 0x38, 0x44, 0x38)
            dms.update_display()
            time.sleep(5)
        except KeyboardInterrupt:
            dms.display_buf = [0]*16
            dms.update_display()
            break