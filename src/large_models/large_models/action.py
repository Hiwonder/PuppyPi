#!/usr/bin/env python3
# coding=utf8

import os
import rclpy
from rclpy.node import Node
from puppy_control_msgs.srv import SetRunActionName
from puppy_control_msgs.msg import Velocity, Pose, Gait
import time
import math
import subprocess
class PuppyControlNode(Node):
    def __init__(self):
        super().__init__('puppy_control_node')
        
        self.pose_publisher = self.create_publisher(Pose, '/puppy_control/pose', 10)
        self.gait_publisher = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.velocity_publisher = self.create_publisher(Velocity, '/puppy_control/velocity', 10)
        self.run_action_group_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup')
        while not self.run_action_group_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /puppy_control/runActionGroup not available, waiting...')
    
    def set_run_action(self, action_name):
        msg = SetRunActionName.Request()
        msg.name = f'{action_name}.d6ac'
        msg.wait = True
        self.run_action_group_srv.call_async(msg)

    def set_move(self, x=0.00, y=0.0, yaw_rate=0.0):
        velocity_msg = Velocity(x=x, y=y, yaw_rate=yaw_rate)
        self.velocity_publisher.publish(velocity_msg)

    def forward(self):
        self.set_move(x=5.0)
        time.sleep(2)
        self.set_move(x=0.0)

    def back(self):
        self.set_move(x=-5.0)
        time.sleep(2)
        self.set_move(x=0.0)

    def turn_left(self):
        self.set_move(x=-5.0, yaw_rate=math.radians(-30))
        time.sleep(2)
        self.set_move(x=0.0, yaw_rate=math.radians(0))

    def turn_right(self):
        self.set_move(x=-5.0, yaw_rate=math.radians(30))

        time.sleep(2)
        self.set_move(x=0.0, yaw_rate=math.radians(0))
        
    # def man(self):
    #     subprocess.run(["paplay", "--stream-name=man_audio", "--device=default", 
    #                 "/home/ubuntu/ros2_ws/src/large_models/large_models/large_models/resources/audio/man.wav"])

    # def woman(self):
    #     subprocess.run(["paplay", "--stream-name=woman_audio", "--device=default", 
    #                 "/home/ubuntu/ros2_ws/src/large_models/large_models/large_models/resources/audio/schoolgirl.wav"])
    def man(self):
        try:
            subprocess.run(
                ["paplay", "--stream-name=man_audio", "--device=default",
                "/home/ubuntu/ros2_ws/src/large_models/large_models/large_models/resources/audio/man.wav"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error playing man.wav: {e}")

    def woman(self):
        try:
            subprocess.run(
                ["paplay", "--stream-name=woman_audio", "--device=default",
                "/home/ubuntu/ros2_ws/src/large_models/large_models/large_models/resources/audio/schoolgirl.wav"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error playing schoolgirl.wav: {e}")

    def init(self):
        self.set_run_action('stand')

    def boxing(self):
        self.set_run_action('boxing')

    def bow(self):
        self.set_run_action('bow')

    def push_up(self):
        self.set_run_action('push-up01')

    def shake_hands(self):
        self.set_run_action('shake_hands')

    def shake_hand(self):
        self.set_run_action('shake_hand')

    def lie_down(self):
        self.set_run_action('lie_down')

    def moonwalk(self):
        self.set_run_action('moonwalk')

    def kick_ball_left(self):
        self.set_run_action('kick_ball_left')

    def kick_ball_right(self):
        self.set_run_action('kick_ball_right')

    def nod(self):
        self.set_run_action('nod')

    def wave(self):
        self.set_run_action('wave')

    def temp(self):
        self.set_run_action('temp')

    def stand(self):
        self.set_run_action('stand')

    def sit(self):
        self.set_run_action('sit')
        
    def kick_ball(self, color='red'):
        os.system('ros2 run example kick_ball_demo' + ' ' + color)
        
    def visual_patrol(self, color='red'):
        os.system('ros2 run example visual_patrol_demo' + ' ' + color)
       
        
def main(args=None):
    rclpy.init(args=args)

    puppy_control_node = PuppyControlNode()

    try:
        rclpy.spin(puppy_control_node)
    except KeyboardInterrupt:
        puppy_control_node.get_logger().info('Node terminated by user')
    finally:
        puppy_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
        

   
