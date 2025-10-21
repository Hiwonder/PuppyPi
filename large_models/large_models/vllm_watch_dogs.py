#!/usr/bin/env python3
# encoding: utf-8
# @Author: liang
# @Date: 2024/12/18
import cv2
import sys
import os
import math
import json
import time
import queue
import rclpy
import time
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
#from track_anything import ObjectTracker
from large_models_msgs.srv import SetString
from puppy_control_msgs.srv import SetRunActionName
sys.path.append('/home/ubuntu/software/puppypi_control/') 
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup
PROMPT = '''
你作为智能机器人助手，可以实时识别图像中的障碍物，并在 `response` 中输出物品名称及相关提示信息。
规则：
1. 如果识别到障碍物：
   - `object` 返回识别出的物品名称。
   - 在 `response` 中提示用户障碍物的位置，并关心提醒注意安全。
   - 如果指令包含动作需求，需根据指令提供对应的 `action`，函数名称只能从以下列表中选择：
     * 站立：stand()
     * 打拳：boxing()
     * 俯卧撑：push_up()
     * 握手：shake_hands()
     * 点头：nod()
     * 坐下：sit()
     * 踢左脚：kick_ball_left()
     * 踢右脚：kick_ball_right()
     * 跳舞：moonwalk()
     * 趴下：lie_down()
     * 伸懒腰：temp()
     * 撒娇：bow()
     * 叫一声：man()
   - 示例：
     如果指令为：  
     "看看有没有什么障碍物，如果有障碍物的话叫一声提醒我"  
     你返回：
     ```json
     {
      "type": "detect",
      "object": "障碍物名称",
      "action": ["man()"],
      "Play": ["detect"],
      "response": "门口有障碍物喔，叫一声提醒你了要注意避开喔"
     }
     ```

2. 如果没有识别到障碍物：
   - `object` 返回 `"None"`。
   - `action` 返回 `["None"]`。
   - 在 `response` 中提示用户环境中没有障碍物，并提醒注意安全。
   - 示例：
     如果指令为：  
     "我现在要出门了，帮我去门口看看有没有什么障碍物"  
     你返回：
     ```json
     {
      "type": "detect",
      "object": "None",
      "action": ["None"],
      "Play": ["detect"],
      "response": "门口没有障碍物喔，注意安全早点回家喔"
     }
     ```

3. 任何 `action` 中的函数名称必须从列表中选择，如果无法匹配到任何动作，返回 `["None"]`。

只需要按照以上规则，返回 JSON 格式的内容即可，不要添加其它内容。,不能乱做动作需要根据提示做动作，叫一声也是动作，也是要执行的

'''

class VLLMTrack(Node):
    def __init__(self, name):
        super().__init__(name)
        self.image_queue = queue.Queue(maxsize=2)
        self.bridge = CvBridge()
        self.action = []
        self.vllm_result = ''
        self.running = True
        #self.track = ObjectTracker()
        #self.vllm = speech.YiVLLM(lingyi_api_key, lingyi_base_url)
        self.yilm = speech.YiLM(lingyi_api_key, lingyi_base_url)
        self.puppy_control_node = PuppyControlNode()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
        self.run_action_group_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup')
        self.set_model_client = self.create_client(SetString, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_vllm_prompt_client = self.create_client(SetString, 'agent_process/set_vllm_prompt')
        self.set_vllm_prompt_client.wait_for_service()
        self.cli = self.create_client(Empty,'/puppy_control/go_home')

        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def init_process(self):
        self.timer.cancel()
        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_vllm_prompt_client, msg)
        msg = SetString.Request()
        msg.data = 'vllm'
        self.send_request(self.set_model_client, msg)
        runActionGroup('look_down.d6ac', True)
        #speech.play_audio(start_audio_path,volume=10)
        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        #self.cli.call_async(Empty.Request())


    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()


    def vllm_result_callback(self, msg):
        """纯回调函数"""
        self.vllm_result = msg.data  # 仅存储接收到的数据
        self.get_logger().info("接收到 VLLM 结果回调")
        self.handle_vllm_result()  # 调用处理函数

    def handle_vllm_result(self):
        """处理 VLLM 结果的逻辑，包括动作解析和响应发布"""  
        if not self.vllm_result:
            return

        try:
            # 提取 JSON 字符串并解析
            result_json = self.vllm_result[self.vllm_result.find('{'):self.vllm_result.find('}') + 1]
            result = eval(result_json)

            action_list = result.get('action', [])
            response = result.get('response', '')

            # 执行动作
            for action in action_list:
                if action and action != "None":
                    try:
                        eval(f'self.puppy_control_node.{action}')  # 执行动作
                    except Exception as e:
                        self.get_logger().error(f"执行动作 '{action}' 失败: {e}")

            # 发布中文响应消息
            time.sleep(6)
            response_msg = String()
            response_msg.data = response
            self.tts_text_pub.publish(response_msg)
            self.cli.call_async(Empty.Request())
            time.sleep(3)
            runActionGroup('look_down.d6ac', True)

        except Exception as e:
            self.get_logger().error(f"处理 VLLM 结果出错: {e}")
        finally:
            self.vllm_result = ''  # 清空结果，避免重复处理

    def process(self):
            while self.running:
                image = self.image_queue.get(block=True)
                if self.vllm_result != '':
                    msg = String()
                    msg.data = self.vllm_result
                    #self.tts_text_pub.publish(msg)
                    self.vllm_result = ''

                cv2.imshow('image', image)
                cv2.waitKey(1)
            cv2.destroyAllWindows()

    def image_callback(self, ros_image):
        try:
            # 使用 cv_bridge 将 ROS Image 转换为 OpenCV 图像
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')

            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像
                self.image_queue.get()

            # 将图像放入队列
            self.image_queue.put(rgb_image)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main():
    rclpy.init()
    node = VLLMTrack('vllm_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node')
    finally:
        node.running = False
        node.destroy_node()


if __name__ == "__main__":
    main()
