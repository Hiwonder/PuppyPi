#!/usr/bin/env python3
# coding=utf8

import sys
import math
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from puppy_control_msgs.msg import Velocity, Pose, Gait

ROS_NODE_NAME = 'puppy_demo'

PuppyMove = {'x': 5.0, 'y': 0.0, 'yaw_rate': 0.0}

PuppyPose = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000, 'height': -10.0, 'x_shift': 0.5, 'stance_x': 0.0, 'stance_y': 0.0}

gait = 'Trot'

if gait == 'Trot':
    GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.3, 'clearance_time': 0.0, 'z_clearance': 5.0}
    PuppyPose['x_shift'] = -0.6
elif gait == 'Amble':
    GaitConfig = {'overlap_time': 0.1, 'swing_time': 0.2, 'clearance_time': 0.1, 'z_clearance': 5.0}
    PuppyPose['x_shift'] = -0.9
elif gait == 'Walk':
    GaitConfig = {'overlap_time': 0.1, 'swing_time': 0.2, 'clearance_time': 0.3, 'z_clearance': 5.0}
    PuppyPose['x_shift'] = -0.65


class PuppyDemoNode(Node):
    def __init__(self):
        super().__init__(ROS_NODE_NAME)
        self.PuppyPosePub = self.create_publisher(Pose, '/puppy_control/pose', 10)
        self.PuppyGaitConfigPub = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.PuppyVelocityPub = self.create_publisher(Velocity, '/puppy_control/velocity', 10)

        self.set_mark_time_client = self.create_client(SetBool, '/puppy_control/set_mark_time')

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.publish_initial_commands()

    def publish_initial_commands(self):
        self.PuppyPosePub.publish(Pose(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                                       height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500))
        self.PuppyGaitConfigPub.publish(Gait(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
                                             clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance']))
        self.PuppyVelocityPub.publish(Velocity(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate']))

        while not self.set_mark_time_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.set_mark_time_client.call_async(SetBool.Request(data=False))
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call succeeded')
        else:
            self.get_logger().info('Service call failed')

    def timer_callback(self):
        if rclpy.ok():
            pass  
        else:
            sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    puppy_demo_node = PuppyDemoNode()
    
    try:
        rclpy.spin(puppy_demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        puppy_demo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

