#!/usr/bin/python3
#coding=utf8
# 第3课 触摸传感器检测(Lesson 3 Touch Sensor Detection)
import os
import sys
import rclpy
import gpiod
import signal
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState


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
# 注册gpio引脚
touch_pin = 22
chip = gpiod.chip("gpiochip4")    
touch = chip.get_line(touch_pin)
config = gpiod.line_request()
# 配置为上拉输入
config.consumer = "touch"
config.request_type = gpiod.line_request.DIRECTION_INPUT
config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
touch.request(config)


class TouchNode(Node):
    def __init__(self):
        # 初始化节点(initialization node)
        super().__init__('touch_control_buzzer')
        signal.signal(signal.SIGINT, self.stop)
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) 
        self.st = 0

    def stop(self):
        self.get_logger().info('Shutting down...')
        self.buzzer_pub.publish(0)
      
    def timer_callback(self):
        state = touch.get_value()
        if not state:
            if self.st == 1: #这里做一个判断，防止反复响(implement a check here to prevent repeated responses)
                self.st = 0
                msg = BuzzerState()
                msg.freq = 1900 
                msg.on_time = 0.5
                msg.off_time = 0.5
                msg.repeat = 1
                self.buzzer_pub.publish(msg)# 蜂鸣器响0.5秒(buzzer emits for 0.5 second)
                self.get_logger().info("Buzzer on")
        else:
            self.st = 1 
            self.get_logger().info("Buzzer off")

        
def main(args=None):
    rclpy.init(args=args)
    node = TouchNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("intteruprt------------")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 

if __name__ == '__main__':
    main()
