#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
from puppy_control.srv import SetRunActionName, SetRunActionNameRequest
from puppy_control.msg import Velocity, Pose, Gait
import time
import math
import subprocess

class PuppyControlNode(object):
    def __init__(self):
        # 不再初始化节点
        # rospy.init_node('puppy_control_node', anonymous=False)
        
        # 发布者
        self.pose_publisher = rospy.Publisher('/puppy_control/pose', Pose, queue_size=10)
        self.gait_publisher = rospy.Publisher('/puppy_control/gait', Gait, queue_size=10)
        self.velocity_publisher = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=10)
        
        # 服务客户端
        rospy.wait_for_service('/puppy_control/runActionGroup')
        try:
            self.run_action_group_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
        except rospy.ServiceException as e:
            rospy.logerr("Service initialization failed: %s", e)
        
    def set_run_action(self, action_name):
        try:
            req = SetRunActionNameRequest()
            req.name = f'{action_name}.d6ac'
            req.wait = True
            self.run_action_group_srv.call(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)

    def set_move(self, x=0.00, y=0.0, yaw_rate=0.0):
        velocity_msg = Velocity()
        velocity_msg.x = x
        velocity_msg.y = y
        velocity_msg.yaw_rate = yaw_rate
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
        
    def man(self):
        try:
            subprocess.run(
                ["paplay", "--stream-name=man_audio", "--device=default",
                "/home/ubuntu/puppypi/src/large_models/scripts/resources/audio/man.wav"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr("播放 man.wav 失败: %s", e)

    def woman(self):
        try:
            subprocess.run(
                ["paplay", "--stream-name=woman_audio", "--device=default",
                "/home/ubuntu/puppypi/src/large_models/scripts/resources/audio/schoolgirl.wav"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True
            )
        except subprocess.CalledProcessError as e:
            rospy.logerr("播放 schoolgirl.wav 失败: %s", e)

    def init(self):
        self.set_run_action('stand')

    def boxing(self):
        self.set_run_action('boxing')

    def bow(self):
        self.set_run_action('bow')

    def push_up(self):
        self.set_run_action('push-up')

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
        try:
            subprocess.run(['rosrun', 'example', 'kick_ball_demo', color],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr("执行 kick_ball_demo 失败: %s", e)
            
    def visual_patrol(self, color='red'):
        try:
            subprocess.run(['rosrun', 'example', 'visual_patrol_demo', color],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        except subprocess.CalledProcessError as e:
            rospy.logerr("执行 visual_patrol_demo 失败: %s", e)
