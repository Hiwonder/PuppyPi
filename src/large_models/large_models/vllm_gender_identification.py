#!/usr/bin/env python3
# encoding: utf-8
# @Author: liang
# @Date: 2024/12/18
import cv2
import math
import json
import time
import queue
import rclpy
import threading
from config import *
import numpy as np
import sdk.fps as fps
from sdk import common
from  action import PuppyControlNode  
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
import subprocess
from speech import speech
from large_models.config import *
#from track_anything import ObjectTracker
from large_models_msgs.srv import SetString
from puppy_control_msgs.srv import SetRunActionName

PROMPT = '''
你作为智能机器人助手，可以识别出图像中的内容，并根据我的描述识别是男生还是女生。
输出要求：
- 返回一个 JSON 对象，包含以下字段：
  - "type": 始终为 "detect"
  - "object": 识别到的目标物体名称，若未识别到则为 "None"
  - "action": 键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]
  - "response": 根据识别到目标是man还是women进行回复
如果指令包含动作需求，需根据指令提供对应的 `action`（按执行顺序排列的函数名称字符串数组），函数名称只能从以下列表中选择：
* 站立：stand()
* 打拳：boxing()
* 俯卧撑：push_up()
* 打招呼: shake_hands()
* 点头：nod()
* 坐下：sit()
* 踢左脚：kick_ball_left()
* 踢右脚：kick_ball_right()
* 跳舞：moonwalk()
* 趴下：lie_down()
* 伸懒腰：temp()
* 撒娇：bow()
*识别到男生的叫一下:man()
*识别到女生的叫一下:woman()
如果识别到目标为"男"，object:"man", 
如果识别到目标为"女"，object:"women"

如果并没有识别到男或者女，则需要说明面前没有男生或者女生需要回复"object": "None","action": ["None"],
只需要返回检测到的目标物体的名称以及动作执行结果。
response是固定返回如果object:"man"，response：面前是个男孩子喔，要我去咬他吗，如果object:"women"，response ：是女孩子要我去打个招呼吗
例如，如果我的指令是：
"你面前的是男生还是女生"
你输出：
{
 "type": "detect",
 "object": "man",
 "action": ["man()"],
 "response": "面前是个男孩子喔，要我去咬他吗"
}
如果指令要求多个动作，例如：
"你面前的男生,做两个俯卧撑，然后叫一下"
你输出：
{
 "type": "detect",
 "object": "man",
 "action": ["push_up()", "push_up()，man()"],
 "response": "面前是个男孩子喔，要我去咬他吗"
}
例如，如果我的指令是：
"你面前的是男生还是女生"
你输出：
{
 "type": "detect",
 "object": "women",
 "action": ["woman()"],
 "response": "面前是个男孩子喔，要我去咬他吗"
}
只回复json本身即可，不要回复其它内容。不能乱做动作需要根据提示做动作，叫一声也是动作，也是要执行的，并且response是一定要返回输出，不管是那个response都需要返回
'''

class VLLMTrack(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.action_done_event = threading.Event()
        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
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
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
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

        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.cli.call_async(Empty.Request())

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def vllm_result_callback(self, msg):
        """纯回调函数，仅接收数据并触发处理逻辑"""
        self.vllm_result = msg.data  # 仅存储接收到的数据
        self.get_logger().info("接收到 VLLM 结果回调")
        self.handle_vllm_result()  # 调用独立的处理函数

    def handle_vllm_result(self):
        """处理 VLLM 结果的逻辑，包括解析动作和发布响应"""
        if not self.vllm_result:
            return  # 如果没有结果，直接返回

        try:
            # 提取 JSON 字符串并解析
            start_idx = self.vllm_result.find('{')
            end_idx = self.vllm_result.find('}') + 1
            if start_idx == -1 or end_idx == 0:
                raise ValueError("无效的 JSON 格式")
            result_json = self.vllm_result[start_idx:end_idx]
            result = json.loads(result_json)  # 使用 json 解析更安全

            action_list = result.get('action', [])
            response = result.get('response', '')

            # 发布中文响应消息
            self.publish_response(response)

            # 等待6秒
            time.sleep(6)

            # 执行动作
            self.execute_actions(action_list)

        except Exception as e:
            self.get_logger().error(f"处理 VLLM 结果时出错: {e}")
        finally:
            self.vllm_result = ''  # 清空结果，避免重复处理

    def execute_actions(self, action_list):
        """执行动作列表中的动作"""
        for action in action_list:
            if action and action != "None" and isinstance(action, str):
                try:
                    eval(f'self.puppy_control_node.{action}')  # 执行动作
                    self.get_logger().info(f"成功执行动作: {action}")
                    self.cli.call_async(Empty.Request())
                except Exception as e:
                    self.get_logger().error(f"执行动作 '{action}' 失败: {e}")
            else:
                self.get_logger().warn(f"无效的动作: {action}，跳过执行。")

    def publish_response(self, response):
        """发布中文响应消息"""
        if response:
            response_msg = String()
            response_msg.data = response
            time.sleep(1)
            self.tts_text_pub.publish(response_msg)
            self.get_logger().info(f"已发布响应消息: {response}")

    def process(self):
        while self.running:
            image = self.image_queue.get(block=True)
            if self.vllm_result != '':
                msg = String()
                msg.data = self.vllm_result
                # self.tts_text_pub.publish(msg)
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
    node = VLLMTrack('vllm_gender_identification')
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
