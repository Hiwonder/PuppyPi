#!/usr/bin/python3
# coding=utf8

import math
import time
import rospy
import threading
import numpy as np
from std_msgs.msg import Float32
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from ros_robot_controller.msg import BuzzerState

MAX_SCAN_ANGLE = 360 # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of the laser, excluding the consistently obstructed portion, in degrees)

class LidarController:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True)
        rospy.on_shutdown(self.cleanup)
        self.name = name
        self.running_mode = 1 # 1：雷达避障模式  2：雷达警卫模式，(1: Radar obstacle avoidance mode 2: Radar guard mode)
        self.threshold = 0.3 # meters  距离阈值(distance threshold)
        self.scan_angle = math.radians(90)  # radians  向前的扫描角度(the forward scanning angle)
        self.speed = 0.12 # 单位米，避障模式的速度(the speed in meters per second for obstacle avoidance mode)
        self.last_act = 0
        self.timestamp = 0
        self.lock = threading.RLock()
        self.lidar_sub = None
        self.velocity_pub = rospy.Publisher('/cmd_vel_nav', geo_msg.Twist, queue_size=1)
        self.velocity_pub.publish(geo_msg.Twist())
        self.lidar_sub = rospy.Subscriber('/scan', sensor_msg.LaserScan, self.lidar_callback) 
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
        self.buzzer_pub.publish(0.1)
        
    def lidar_callback(self, lidar_data:sensor_msg.LaserScan):
        twist = geo_msg.Twist()
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]
        right_ranges = lidar_data.ranges[::-1][:max_index]
        
        with self.lock:
            #angle = math.atan(self.car_width / self.threshold)
            angle = self.scan_angle / 2
            angle_index = int(angle / lidar_data.angle_increment + 0.50)
            
            left_ranges, right_ranges = left_ranges[:angle_index], right_ranges[:angle_index]
            
            if self.running_mode == 1 and self.timestamp <= time.time():
                min_index_left, min_index_right = np.nanargmin(np.array(left_ranges)),  np.nanargmin(np.array(right_ranges))
                min_dist_left, min_dist_right = left_ranges[min_index_left], right_ranges[min_index_right]
                angle_left = lidar_data.angle_increment * min_index_left
                angle_right = lidar_data.angle_increment * min_index_right
                # print(min_dist_left, min_dist_right)
                if min_dist_left <= self.threshold and min_dist_right > self.threshold: # 左侧有障碍(there is an obstacle on the left side)
                    twist.linear.x = self.speed / 6
                    max_angle = math.radians(90)
                    w = self.speed * 3
                    twist.angular.z = -w
                    self.velocity_pub.publish(twist)
                    self.timestamp = time.time() + (max_angle / w / 2)
                elif min_dist_left <= self.threshold and min_dist_right <= self.threshold: # 两侧都有障碍(there are obstacles on both sides)
                    twist.linear.x = self.speed / 6
                    w = self.speed * 3
                    twist.angular.z = w
                   
                    self.velocity_pub.publish(twist)
                    self.timestamp = time.time() + (math.radians(30) / w / 2)
                elif min_dist_left > self.threshold and min_dist_right <= self.threshold: # 右侧有障碍(there is an obstacle on the right side)
                    twist.linear.x = self.speed / 6
                    max_angle = math.radians(90)
                    w = self.speed * 3
                    twist.angular.z = w
                    
                    self.velocity_pub.publish(twist)
                    self.timestamp = time.time() + (max_angle / w /2)
                else: # 没有障碍(there are no obstacles)
                    self.last_act = 0
                    twist.linear.x = self.speed
                    twist.angular.z = 0
                    self.velocity_pub.publish(twist)

            elif self.running_mode == 2:
                min_index_left, min_index_right = np.nanargmin(np.array(left_ranges)),  np.nanargmin(np.array(right_ranges))
                min_dist_left, min_dist_right = left_ranges[min_index_left], right_ranges[min_index_right]
                if min_dist_left <= self.threshold or min_dist_right <= self.threshold: # 检测到障碍(the obstacle is detected)
                    self.buzzer_pub.publish(0.1)

    def cleanup(self):
       
        self.lidar_sub.unregister()
        self.velocity_pub.publish(geo_msg.Twist())
        print('is_shutdown')


if __name__ == "__main__":
    node = LidarController('lidar_demo')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
