#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18

import re
import time
import rclpy
import threading
import json
from config import *
from speech import speech
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

from large_models.config import *
from large_models_msgs.srv import SetLLM, SetColor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class LLMColorSorting(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.action = []
        self.llm_result = ''
        self.running = True
        
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 10)
        self.create_subscription(String, '/agent_process/result', self.llm_result_callback, 10)

        self.set_llm_client = self.create_client(SetLLM, 'agent_process/set_llm')
        if not self.set_llm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'agent_process/set_llm' not available.")
        
        self.enter_client = self.create_client(Trigger, '/color_tracking/enter')
        if not self.enter_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/color_tracking/enter' not available.")

        self.start_client = self.create_client(SetBool, '/color_tracking/enable_color_tracking')
        if not self.start_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/color_tracking/enable_color_tracking' not available.")

        self.set_color_client = self.create_client(SetColor, '/color_tracking/set_color')
        if not self.set_color_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/color_tracking/set_color' not available.")

        self.stop_client = self.create_client(Trigger, '/color_tracking/stop')
        if not self.stop_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/color_tracking/stop' not available.")

        # 初始化线程锁
        self.llm_result_lock = threading.Lock()

        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.1, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def init_process(self):
        self.timer.cancel()

        # 调用 SetLLM 服务
        msg = SetLLM.Request()
        msg.app_id = '04c230e9-2267-4831-b861-4a77f6f9be68'
        msg.conversation_id = 'c99865f6-00e7-4103-b381-dd2ea9fc59e7'
        response = self.send_request(self.set_llm_client, msg)
        if response:
            self.get_logger().info("SetLLM service call succeeded.")
        else:
            self.get_logger().error("SetLLM service call failed.")

        # 调用 /color_tracking/enter 服务
        response = self.send_request(self.enter_client, Trigger.Request())
        if response:
            self.get_logger().info("Entered color tracking.")
        else:
            self.get_logger().error("Failed to enter color tracking.")

        # 播放音频
        if 'start_audio_path' in globals():
            speech.play_audio(start_audio_path)
        else:
            self.get_logger().error("start_audio_path 未定义。")
        
        # 启动处理线程
        threading.Thread(target=self.process, daemon=True).start()

        # 创建服务
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                    self.get_logger().info("Service call succeeded.")
                    return response
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {e}")
                    return None
            time.sleep(0.1)  # Prevent busy-waiting
        self.get_logger().error("ROS2 not ok, service call aborted.")
        return None

    def llm_result_callback(self, msg):
        with self.llm_result_lock:
            self.llm_result += msg.data  # 追加接收到的数据
        self.get_logger().info(f"Received data: {msg.data}")  # 使用 info 级别日志

    def process(self):
        buffer = ""
        while self.running:
            with self.llm_result_lock:
                buffer += self.llm_result
                self.llm_result = ''
            if buffer:
                # 尝试从缓冲区中提取所有完整的 JSON 对象
                while True:
                    json_str = self.extract_json(buffer)
                    if json_str:
                        try:
                            result = json.loads(json_str)
                            self.handle_result(result)
                            # 移除已处理的 JSON
                            buffer = buffer.replace(json_str, '', 1)
                        except json.JSONDecodeError as e:
                            self.get_logger().error(f"JSON decoding failed: {e}")
                            buffer = ''
                            break
                    else:
                        break
                time.sleep(0.1)
            else:
                time.sleep(0.01)

    def extract_json(self, data):
        """
        从数据字符串中提取 JSON 内容。
        移除代码围栏并确保 JSON 完整。
        """
        # 移除代码围栏（如果存在）
        data = re.sub(r'^```json\s*', '', data)
        data = re.sub(r'```$', '', data)

        # 使用正则表达式查找 JSON 对象
        match = re.search(r'({.*})', data, re.DOTALL)
        if match:
            json_candidate = match.group(1)
            # 通过尝试解析来验证 JSON 是否完整
            try:
                json.loads(json_candidate)
                return json_candidate
            except json.JSONDecodeError:
                # JSON 不完整
                return None
        return None

    def handle_result(self, result):
        """
        处理解析后的 JSON 结果。
        """
        if 'action' not in result or not isinstance(result['action'], list):
            self.get_logger().error("Invalid result format: 'action' missing or not a list.")
            return

        if 'grab' in result['action']:
            if 'object' in result and result['object']:
                color = result['object']
                self.get_logger().info(f"Action: grab, Color: {color}")

                # 调用 SetColor 服务设置颜色
                set_color_request = SetColor.Request()
                set_color_request.color = color  # 确保 SetColor.Request 中有 'color' 字段
                response = self.send_request(self.set_color_client, set_color_request)
                if response:
                    self.get_logger().info(f"Set color to {color} succeeded.")
                else:
                    self.get_logger().error(f"Set color to {color} failed.")

                # 调用 SetBool 服务启用颜色跟踪
                set_bool_request = SetBool.Request()
                set_bool_request.data = True
                response = self.send_request(self.start_client, set_bool_request)
                if response:
                    self.get_logger().info("Enabled color tracking.")
                else:
                    self.get_logger().error("Failed to enable color tracking.")

                # 发布 TTS 消息
                if 'response' in result and result['response']:
                    msg = String()
                    msg.data = result['response']
                    self.tts_text_pub.publish(msg)
                    self.get_logger().info(f"Published TTS message: {msg.data}")
            else:
                self.get_logger().error("'grab' action received but 'object' is missing or empty.")

        elif 'stop' in result['action']:
            if 'object' in result and not result['object']:
                self.get_logger().info("Action: stop")

                # 调用 SetBool 服务禁用颜色跟踪
                set_bool_request = SetBool.Request()
                set_bool_request.data = False
                response = self.send_request(self.start_client, set_bool_request)
                if response:
                    self.get_logger().info("Disabled color tracking.")
                else:
                    self.get_logger().error("Failed to disable color tracking.")

                # 调用 Trigger 服务停止颜色跟踪
                stop_request = Trigger.Request()
                response = self.send_request(self.stop_client, stop_request)
                if response:
                    self.get_logger().info("Color tracking stopped.")
                else:
                    self.get_logger().error("Failed to stop color tracking.")

                # 发布 TTS 消息
                if 'response' in result and result['response']:
                    msg = String()
                    msg.data = result['response']
                    self.tts_text_pub.publish(msg)
                    self.get_logger().info(f"Published TTS message: {msg.data}")
            else:
                self.get_logger().error("'stop' action received but 'object' is not empty.")

        elif not result['action'] and 'response' in result:
            # 处理空动作，发布 TTS 消息
            msg = String()
            msg.data = result['response']
            self.tts_text_pub.publish(msg)
            self.get_logger().info(f"Published TTS message: {msg.data}")

        else:
            self.get_logger().warn("Received message with unhandled 'action'.")


    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main():
    rclpy.init()
    node = LLMColorSorting('llm_tracking')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
