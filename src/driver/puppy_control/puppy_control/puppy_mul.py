#!/usr/bin/env python3
# coding=utf8
# Author: Summer
# Email: 997950600@qq.com

import os
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

sys.path.append('/home/ubuntu/software/puppypi_control')
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup

ROS_NODE_NAME = 'puppy_control'


class PUPPY(Node):
    def __init__(self):
        super().__init__('puppy_run_act')
        self.subscription = self.create_subscription(
            String,
            '/multi_robot/runActionGroup',
            self.runActionGroupFun,
            10
        )
        self.subscription  # prevent unused variable warning

    def runActionGroupFun(self, msg):
        self.get_logger().debug(f'Received message: {msg.data}')
        print(msg.data)
        runActionGroup(msg.data, False)
        return True


def main(args=None):
    rclpy.init(args=args)
    puppy = PUPPY()

    try:
        rclpy.spin(puppy)
    except KeyboardInterrupt:
        pass
    finally:
        puppy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

