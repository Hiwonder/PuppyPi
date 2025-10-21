#!/usr/bin/env python3
# 第2课 机器狗超声波测距避障(Lesson 2 Robot Dog Ultrasonic Distance Measurement and Obstacle Avoidance)
import os
import sys
import math
import rclpy
import signal
import time
from rclpy.node import Node
import sdk.sonar as sonar
from puppy_control_msgs.msg import Velocity, Pose, Gait

print('''
**********************************************************
********************功能:超声波避障例程(function: ultrasonic obstacle avoidance routine)**********************
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

PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10.0, 'x_shift':0.0, 'stance_x':0.0, 'stance_y':0.0}
# stance_x：4条腿在x轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the X-axis, measured in centimeters)
# stance_y：4条腿在y轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the Y-axis, measured in centimeters)
# x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm(the distance traveled by the four legs along the x-axis determines the degree of forward or backward tilt during walking: smaller distances lead to more forward tilt, while larger distances result in more backward tilt. Adjusting the x_shift parameter can help maintain balance during the dog's movement, measured in centimeters)
# height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm(the height of the dog, measured from the toe to the axis  of rotation of the thigh, is in centimeters)
# pitch： 狗身体的俯仰角，单位弧度(the pitch angle of the dog's body, measured in radians)


GaitConfig = {'overlap_time':0.15, 'swing_time':0.2, 'clearance_time':0.0, 'z_clearance':3.0}
# overlap_time:4脚全部着地的时间，单位秒(the time when all four legs touch the ground, measured in seconds)
# swing_time：2脚离地时间，单位秒(the time duration when legs are off the ground, measured in second)
# clearance_time：前后脚间隔时间，单位秒(the time interval between the front and rear legs, measured in seconds)
# z_clearance：走路时，脚抬高的距离，单位cm(the distance the paw needs to be raised during walking, measured in centimeters)    
    
class SonarAvoidance(Node):
    def __init__(self):
        super().__init__('sonar_avoidance')
        signal.signal(signal.SIGINT, self.stop)
        self.s = sonar.Sonar()
        self.forward = True       
        self.pose_publisher = self.create_publisher(Pose, '/puppy_control/pose',10)
        self.gait_publisher = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.velocity_publisher = self.create_publisher(Velocity, '/puppy_control/velocity',10)
        self.pose_publisher.publish(Pose(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                                       height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500))
        
        self.gait_publisher.publish(Gait(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance']))        
          
        
        self.timer = self.create_timer(0.1, self.sonar_callback)  
     
    def sonar_callback(self):
        distance = self.s.getDistance() # 获得检测的距离(obtain detected distance)
        print(f'distance: {distance}(mm)')
        
        if distance <= 300:
            if not self.forward:
                self.forward = True
                self.s.setRGB(1, (255, 0, 0)) # 设为红色(set to red color)
                self.s.setRGB(0, (255, 0, 0))
                # 设置 RGB 灯颜色 (set RGB light color)
                #self.velocity_publisher.publish(Velocity(x=5.0, y=0.0, yaw_rate=0.3))
                self.get_logger().info('Turning left')               
        else:
            if self.forward:
                self.forward = False
                # 设置 RGB 灯颜色 (set RGB light color)
                self.s.setRGB(1, (0, 0, 255)) # 设为蓝色(set to blue)
                self.s.setRGB(0, (0, 0, 255))
                self.velocity_publisher.publish(Velocity(x=15.0, y=0.0, yaw_rate=0.0))
                self.get_logger().info('Moving forward')   
    
                 
    def stop(self):
        self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
        self.get_logger().info('Shutting down...')
        self.s.setRGB(1, (0, 0, 0))
        self.s.setRGB(0, (0, 0, 0))
        
def main(args=None):
    rclpy.init(args=args)
    node = SonarAvoidance()

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