#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import cv2
import sys
import os
import math
import json
import time
import queue
import rclpy
import time
from std_msgs.msg import String, Bool
import threading
from config import *
import numpy as np
import sdk.fps as fps
from sdk import common
from action import PuppyControlNode
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
from speech import speech
from large_models.config import *

from large_models_msgs.srv import SetString
from puppy_control_msgs.srv import SetRunActionName
sys.path.append('/home/ubuntu/software/puppypi_control/') 
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup

class VLLMWithCamera(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.image_queue = queue.Queue(maxsize=2)
        self.bridge = CvBridge()
        self.vllm_result = ''
        self.running = True
        timer_cb_group = ReentrantCallbackGroup()
        self.puppy_control_node = PuppyControlNode()
         
        #self.vllm = speech.VLLM(lingyi_api_key, lingyi_base_url)
        self.yilm = speech.YiLM(lingyi_api_key, lingyi_base_url)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_callback, 1)
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup', callback_group=timer_cb_group)
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetString, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.cli = self.create_client(Empty,'/puppy_control/go_home')
        self.cli.call_async(Empty.Request())
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def init_process(self):
        self.timer.cancel()
        
        msg = SetString.Request()
        msg.data = 'vllm'
        self.send_request(self.set_model_client, msg)
        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def vllm_result_callback(self, msg):
        self.vllm_result = msg.data

    def process(self):
        while self.running:
            image = self.image_queue.get(block=True)
            if self.vllm_result != '':
                msg = String()
                msg.data = self.vllm_result
                self.tts_text_pub.publish(msg)
                self.vllm_result = ''
            cv2.imshow('image', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)
        cv2.destroyAllWindows()

    def play_audio_callback(self, msg):
        if msg.data:
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)

    def image_callback(self, ros_image):
        try:
            # 使用 cv_bridge 将 ROS Image 转换为 OpenCV 图像
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='rgb8')

            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像
                self.image_queue.get()

            # 将图像放入队列
            self.image_queue.put(rgb_image)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main():
    node = VLLMWithCamera('vllm_with_camera')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()
