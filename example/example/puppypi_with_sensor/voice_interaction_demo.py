#!/usr/bin/env python3
# coding=utf8
# 第8课 机器狗语音识别交互(Lesson 8 Robot Dog Voice Recognition and Interaction)
import os
import sys
import rclpy
import math
import signal
import serial
import time
from rclpy.node import Node
from puppy_control_msgs.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************功能:语音交互例程(function: voice interaction routine)*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

GaitConfig = {'overlap_time':0.2, 'swing_time':0.2, 'clearance_time':0.0, 'z_clearance':3.0}
# overlap_time:4脚全部着地的时间，单位秒(the time when all four legs touch the ground, measured in seconds)
# swing_time：2脚离地时间，单位秒(the time duration when legs are off the ground, measured in second)
# clearance_time：前后脚间隔时间，单位秒(the time interval between the front and rear legs, measured in seconds)
# z_clearance：走路时，脚抬高的距离，单位cm(the distance the paw needs to be raised during walking, measured in centimeters)



class VoiceInteractionDemo(Node):
    def __init__(self):
        super().__init__('voice_interaction_demo')
        signal.signal(signal.SIGINT, self.stop)
    
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  
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
                                
        self.timer = self.create_timer(0.1, self.asr_callback)


    def asr_callback(self):
         
        if self.ser.in_waiting > 0:        
            data = self.ser.read(5) 
            hex_data = ' '.join(format(byte, '02X') for byte in data)           
            self.get_logger().info(f"Received data: {hex_data}")
            
            if hex_data == "AA 55 00 8A FB": # 抬头(raise the head)               
                self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                                       height=self.PuppyPose['height'], roll=self.PuppyPose['roll'], pitch=math.radians(20), yaw=self.PuppyPose['yaw'], run_time=500))
            elif hex_data == "AA 55 00 0B FB":# 趴下(lie down)
                self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                                       height=self.PuppyPose['height'] + 4, roll=self.PuppyPose['roll'], pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time=500))
            elif hex_data == "AA 55 00 0A FB":# 立正(stand at attention)
                self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                                       height=self.PuppyPose['height'], roll=self.PuppyPose['roll'], pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time=500))
            elif hex_data == "AA 55 00 76 FB":# 原地踏步(stepping in the place)
                self.pose_publisher.publish(Pose(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift'],
                                       height=self.PuppyPose['height'], roll=self.PuppyPose['roll'], pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time=500))
                time.sleep(0.5)
                self.velocity_publisher.publish(Velocity(x=0.1, y=0.0, yaw_rate=0.0))
                time.sleep(2)
                self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
            elif hex_data == "AA 55 02 00 FB": 
                self.get_logger().info('restart wakeup!!!!!!!!!!!!!!!!!!')
            else:
                time.sleep(0.02)
                                       
        else:
            time.sleep(0.02)
            
    # 关闭检测函数(turn off detection function)       
    def stop(self):
        self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
        self.get_logger().info('Shutting down...')
   

def main(args=None):
    rclpy.init(args=args)
    node = VoiceInteractionDemo()

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
