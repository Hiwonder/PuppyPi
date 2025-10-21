#!/usr/bin/python3
#coding=utf8
# 第5课 语音识别传感器实验(Lesson 5 Voice Recognition Sensor)
import os
import sys
import rclpy
import serial
import binascii
import signal
from rclpy.node import Node
from ros_robot_controller_msgs.msg import RGBState, RGBsState


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


class ASRDetectDemo(Node):
    def __init__(self):
        super().__init__('asr_detect_demo')
        signal.signal(signal.SIGINT, self.stop)       
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)                             
        self.rgb_pub = self.create_publisher(RGBsState, '/ros_robot_controller/set_rgb', 10)
        self.timer = self.create_timer(0.1, self.asr_callback)


    def asr_callback(self):
        if self.ser.in_waiting > 0:
            data = self.ser.read(5)  
            self.parse_serial_data(data)
            
            
    def parse_serial_data(self, data):
        hex_data = ' '.join(format(byte, '02X') for byte in data)
        self.get_logger().info(f"Received data: {hex_data}")
        if hex_data == "AA 55 00 8A FB": 
            self.set_rgb_show(255, 0, 0)
        elif hex_data == "AA 55 00 8B FB":
            self.set_rgb_show(0, 255, 0)
        elif hex_data == "AA 55 00 8C FB":
            self.set_rgb_show(0, 0, 255)
        elif hex_data == "AA 55 00 09 FB": 
            self.turn_off_rgb()
        elif hex_data == "AA 55 02 00 FB": 
            self.get_logger().info('restart wakeup!!!!!!!!!!!!!!!!!!')
            
    # 关闭检测函数(turn off detection function)       
    def stop(self):
        self.turn_off_rgb()
        self.get_logger().info('Shutting down...')
    # 关闭RGB彩灯(turn off color light)    
    def turn_off_rgb(self):
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
        self.rgb_pub.publish(msg)
    # 设置RGB彩灯显示(set the RGB color light to display)    
    def set_rgb_show(self,r, g, b):
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
        self.rgb_pub.publish(msg)       
               


def main(args=None):
    rclpy.init(args=args)
    node = ASRDetectDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

        
           
