#!/usr/bin/env python3
# coding=utf8
# Date:2021/09/27
# Author:hiwonder

import math
import time
import rclpy
from rclpy.node import Node
from threading import Timer, RLock
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool, Trigger
from puppy_control_msgs.srv import *

MAX_SCAN_ANGLE = 360  # 激光的扫描角度 (the scanning angle of the laser)

class LidarController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.running_mode = 0  # 1: 雷达避障模式 2: 雷达警卫模式 (1: Radar obstacle avoidance mode, 2: Radar guard mode)
        self.threshold = 0.9  # meters  距离阈值 (distance threshold)
        self.scan_angle = math.radians(90)  # radians  向前的扫描角度 (the forward scanning angle)
        self.speed = 0.12  # 单位米，避障模式的速度 (speed in meters per second for obstacle avoidance mode)
        self.timestamp = 0
        self.lock = RLock()
        self.lidar_sub = None
        self.heartbeat_timer = None

        # 创建Publisher
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.velocity_pub.publish(Twist())

        # 创建服务
        self.enter_srv = self.create_service(Trigger, '/lidar_app/enter', self.enter_func)
        self.exit_srv = self.create_service(Trigger, '/lidar_app/exit', self.exit_func)
        self.heartbeat_srv = self.create_service(SetBool, '/lidar_app/heartbeat', self.heartbeat_srv_cb)
        self.set_running_srv = self.create_service(SetInt64, "/lidar_app/set_running", self.set_running_srv_callback)
        self.set_parameters_srv = self.create_service(SetFloat64List, "/lidar_app/adjust_parameters", self.set_parameters_srv_callback)

    def reset_value(self):
        self.running_mode = 0
        self.threshold = 0.3
        self.speed = 0.12
        self.scan_angle = math.radians(90)

        if self.lidar_sub is not None:
            self.lidar_sub.destroy()
            self.lidar_sub = None

    def enter_func(self, request, response):
        self.get_logger().info("Lidar entering operation mode")
        self.reset_value()
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        response.success = True
        response.message = 'Entered'
        return response

    def exit_func(self, request, response):
        self.get_logger().info('Lidar exiting operation mode')
        self.reset_value()
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
        response.success = True
        response.message = 'Exited'
        return response

    def heartbeat_srv_cb(self, request, response):
        if self.heartbeat_timer:
            self.heartbeat_timer.cancel()
        if request.data:
            self.heartbeat_timer = Timer(5, self.exit_func, [Trigger.Request(), Trigger.Response()])
            self.heartbeat_timer.start()
        response.success = request.data
        return response

    def lidar_callback(self, lidar_data: LaserScan):
        ranges = list(lidar_data.ranges)
        ranges = [9999.0 if r < 0.05 else r for r in ranges]  # 小于5cm当作无限远 (treat distances less than 5cm as infinity)
        twist = Twist()

        with self.lock:
            min_index = np.nanargmin(np.array(ranges))  # 找出距离最小值 (find out the minimum value of distance)
            dist = ranges[min_index]
            angle = lidar_data.angle_min + lidar_data.angle_increment * min_index  # 计算最小值对应的角度 (calculate the angle corresponding to the minimum value)
            angle = angle if angle < math.pi else angle - math.pi * 2  # 处理角度 (handle angle)

            # 避障 (obstacle avoidance)
            if self.running_mode == 1 and self.timestamp <= time.time():
                if abs(angle) < self.scan_angle / 2 and dist < self.threshold:
                    twist.linear.x = self.speed / 6
                    twist.angular.z = self.speed * 3 * -np.sign(angle)
                    self.timestamp = time.time() + 0.8
                else:
                    twist.linear.x = self.speed
                    twist.angular.z = 0.0
                self.velocity_pub.publish(twist)

            # 追踪 (tracking)
            elif self.running_mode == 2 and self.timestamp <= time.time():
                if abs(angle) < self.scan_angle / 2:
                    if dist < self.threshold and abs(math.degrees(angle)) > 10:  # 控制左右 (control the left and the right)
                        twist.linear.x = 0.01  # x方向的校正 (correction in the x-direction)
                        twist.angular.z = self.speed * 3 * np.sign(angle)
                        self.timestamp = time.time() + 0.4
                    else:
                        if dist < self.threshold and dist > 0.35:
                            twist.linear.x = self.speed
                            twist.angular.z = 0.0
                            self.timestamp = time.time() + 0.4
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                self.velocity_pub.publish(twist)

            # 警卫看守 (guard duty)
            elif self.running_mode == 3 and self.timestamp <= time.time():
                if dist < self.threshold and abs(math.degrees(angle)) > 10:
                    twist.linear.x = 0.01  # x方向的校正 (correction in the x-direction)
                    twist.angular.z = self.speed * 3 * np.sign(angle)
                    self.timestamp = time.time() + 0.4
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                self.velocity_pub.publish(twist)

    def set_running_srv_callback(self, request, response):
        new_running_mode = request.data
        self.get_logger().info(f"Setting running mode to {new_running_mode}")
        if not 0 <= new_running_mode <= 3:
            response.success = False
            response.message = f"Invalid running mode {new_running_mode}"
        else:
            self.running_mode = new_running_mode
            self.velocity_pub.publish(Twist())
            response.success = True
            response.message = f"Running mode set to {new_running_mode}"
        return response

    def set_parameters_srv_callback(self, request, response):
        new_threshold, new_scan_angle, new_speed = request.data
        self.get_logger().info(f"Setting new parameters: threshold={new_threshold:.2f}, scan_angle={new_scan_angle:.2f}, speed={new_speed:.2f}")
        if not 0.3 <= new_threshold <= 1.5:
            response.success = False
            response.message = f"Threshold {new_threshold:.2f} is out of range (0.3 ~ 1.5)"
        elif new_speed <= 0:
            response.success = False
            response.message = "Speed must be greater than 0"
        else:
            self.threshold = new_threshold
            self.scan_angle = math.radians(new_scan_angle)
            self.speed = new_speed * 0.002
            response.success = True
            response.message = "Parameters updated successfully"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LidarController('lidar_app')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
