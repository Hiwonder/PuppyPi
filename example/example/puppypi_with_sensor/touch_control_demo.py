#!/usr/bin/python3
#coding=utf8
# 第6课 触摸检测感应
import os
import sys
import time
import math
import rclpy
import gpiod
import signal
from rclpy.node import Node
from std_msgs.msg import *
from puppy_control_msgs.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************功能:触摸控制例程(function: touch control routine)*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

# 触摸模块接扩展板上的IO22、IO24接口(connect the touch module to the IO22 and IO24 interfaces on expansion board)




GaitConfig = {'overlap_time':0.2, 'swing_time':0.2, 'clearance_time':0.0, 'z_clearance':3.0}
# overlap_time:4脚全部着地的时间，单位秒(the time when all four legs touch the ground, measured in seconds)
# swing_time：2脚离地时间，单位秒(the time duration when legs are off the ground, measured in second)
# clearance_time：前后脚间隔时间，单位秒(the time interval between the front and rear legs, measured in seconds)
# z_clearance：走路时，脚抬高的距离，单位cm(the distance the paw needs to be raised during walking, measured in centimeters)


# 触摸模块接扩展板上的IO22、IO24接口(connect the touch module to the IO22 and IO24 interfaces on the expansion board)
touch_pin = 22
chip = gpiod.chip("gpiochip4")
    
touch = chip.get_line(touch_pin)
config = gpiod.line_request()
config.consumer = "touch"
config.request_type = gpiod.line_request.DIRECTION_INPUT
config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
touch.request(config)


class TouchControl(Node):
    def __init__(self):
        super().__init__('touch_control_demo')
        signal.signal(signal.SIGINT, self.stop)
        self.squat = True   
        self.click = 0   
        self.pose_publisher = self.create_publisher(Pose, '/puppy_control/pose',10)
        self.gait_publisher = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.velocity_publisher = self.create_publisher(Velocity, '/puppy_control/velocity',10)
        
        self.PuppyPose = {'roll':math.radians(4), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10.0, 'x_shift':-0.5, 'stance_x':0.0, 'stance_y':0.0}
        # stance_x：4条腿在x轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the X-axis, measured in centimeters)
        # stance_y：4条腿在y轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the Y-axis, measured in centimeters)
        # x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm(the distance traveled by the four legs along the x-axis determines the degree of forward or backward tilt during walking: smaller distances lead to more forward tilt, while larger distances result in more backward tilt. Adjusting the x_shift parameter can help maintain balance during the dog's movement, measured in centimeters)
        # height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm(the height of the dog, measured from the toe to the axis  of rotation of the thigh, is in centimeters)
        # pitch： 狗身体的俯仰角，单位弧度(the pitch angle of the dog's body, measured in radians)
        
        
        self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                                       height=self.PuppyPose['height'], roll=self.PuppyPose['roll'], pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time=500))
        
        self.gait_publisher.publish(Gait(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance']))   
                  
             
        self.timer = self.create_timer(0.1, self.touch_callback)  
     
    def touch_callback(self):
        self.state = touch.get_value()
        if not self.state:
            detect_time = time.time()+1
            while time.time() < detect_time:
                self.state = touch.get_value() 
                time.sleep(0.1)
                if not self.state:
                   self.click += 1
                   time.sleep(0.2)                     
            if self.click == 1:
                self.click =0
                if self.squat:
                    # 机器狗下蹲(the robot dog crouches down)
                    self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                                       height=self.PuppyPose['height'] + 3.0, roll=self.PuppyPose['roll'], pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time=500))
                    time.sleep(1)
                    self.squat = False
                elif not self.squat:                
                    self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                                       height=self.PuppyPose['height'], roll=self.PuppyPose['roll'], pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time=500))
                    time.sleep(1)
                    self.squat = True
                    
            elif self.click == 2:
                self.click = 0
                # 机器狗抖一抖(the robot dog shakes a little)
                for i in  range(5):
                    self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                        height=self.PuppyPose['height'], roll=math.radians(3.5), pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time = 50))
                    time.sleep(0.13)
                    self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                    height=self.PuppyPose['height'], roll=math.radians(-3.5), pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time = 300))
                    time.sleep(1) 
                
    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
                        
    def stop(self):
        self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
        self.get_logger().info('Shutting down...')

        
def main(args=None):
    rclpy.init(args=args)
    node = TouchControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("intteruprt------------")
        pass

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
 

if __name__ == '__main__':
    main()



