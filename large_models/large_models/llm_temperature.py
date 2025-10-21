#!/usr/bin/env python3
# encoding: utf-8
# @Author: liang
# @Date: 2024/12/03


import smbus
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from large_models_msgs.srv import SetString
from large_models_msgs.srv import SetLLM  
import json
import re
from speech import speech
from puppy_control_msgs.srv import SetRunActionName

class AHT10:
    CONFIG = [0x08, 0x00]
    MEASURE = [0x33, 0x00]

    def __init__(self, bus=1, addr=0x38):
        self.bus = smbus.SMBus(bus)
        self.addr = addr
        time.sleep(0.2)

    def getData(self):
        self.bus.write_i2c_block_data(self.addr, 0xAC, self.MEASURE)
        time.sleep(0.5)
        data = self.bus.read_i2c_block_data(self.addr, 0x00)
        temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        ctemp = ((temp * 200) / 1048576) - 50
        return ctemp


class LLMControlServo(Node):
    def __init__(self, name):
        super().__init__(name)

        # 初始化温度传感器
        self.aht10 = AHT10()

        # 创建发布者
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 10)

        # 创建订阅者
        self.subscription = self.create_subscription(
            String,
            '/agent_process/result',  # 使用绝对名称，确保正确订阅
            self.llm_result_callback,
            10
        )
        self.cli = self.create_client(Empty,'/puppy_control/go_home')
        # 创建服务客户端
        self.set_llm_client = self.create_client(SetLLM, '/agent_process/set_llm')
        self.set_llm_content_client = self.create_client(SetString, '/agent_process/set_llm_content')

        # 等待服务可用
        self.get_logger().info('等待 /agent_process/set_llm 服务可用...')
        while not self.set_llm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /agent_process/set_llm 服务可用...')
        self.get_logger().info('/agent_process/set_llm 服务已可用')

        self.get_logger().info('等待 /agent_process/set_llm_content 服务可用...')
        while not self.set_llm_content_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /agent_process/set_llm_content 服务可用...')
        self.get_logger().info('/agent_process/set_llm_content 服务已可用')

        # 配置 LLM（调用 set_llm 服务）
        self.configure_llm(app_id="9a4f9740-bb98-4738-8146-b0502c641d961",
                          conversation_id="675f7fd7-3e8b-447f-bc8b-8079787134891")

        self.get_logger().info('LLMControlServo 节点已启动，等待消息...')

        # 标志位，防止同时处理多个请求
        self.processing = False

    def configure_llm(self, app_id, conversation_id):
        """
        异步调用 /agent_process/set_llm 服务配置 LLM。
        """
        self.get_logger().info(f"配置 LLM: app_id={app_id}, conversation_id={conversation_id}")

        request = SetLLM.Request()
        request.app_id = app_id
        request.conversation_id = conversation_id

        future = self.set_llm_client.call_async(request)
        future.add_done_callback(self.configure_llm_callback)
        start_audio_path = "/home/ubuntu/ros2_ws/src/large_models/large_models/resources/audio/start_audio.wav"
        speech.play_audio(start_audio_path)
        self.cli.call_async(Empty.Request())
    def configure_llm_callback(self, future):
        """
        配置 LLM 服务的回调函数。
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"LLM 配置成功: {response.message}")
            else:
                self.get_logger().error(f"LLM 配置失败: {response.message}")
        except Exception as e:
            self.get_logger().error(f"调用 /agent_process/set_llm 服务时出错: {e}")

    def get_temperature(self):
        """
        获取当前温度并返回。
        """
        try:
            temperature = self.aht10.getData()
            self.get_logger().info(f"当前温度: {temperature:.2f} °C")
            return temperature
        except Exception as e:
            self.get_logger().error(f"温度传感器读取失败: {e}")
            return None

    def llm_result_callback(self, msg):
        """
        处理从 /agent_process/result 接收到的结果。
        """
        self.get_logger().info(f"收到 /agent_process/result 消息: {msg.data}")

        try:
            # 去除代码块标记 ```json 和 ```
            cleaned_data = self.clean_json_message(msg.data)
            if not cleaned_data:
                self.get_logger().error("清理后的数据为空，无法解析。")
                return

            # 解析 JSON 数据
            result = json.loads(cleaned_data)
            if not isinstance(result, dict):
                self.get_logger().error("接收到的 JSON 数据不是一个对象。")
                return

            action_list = result.get('action', [])
            response = result.get('response', '')

            # 如果 'action' 字段存在且包含 "get_temperature()", 则是请求消息
            if "get_temperature()" in action_list:
                if self.processing:
                    self.get_logger().warn("当前正在处理一个请求，忽略新的请求。")
                    return

                self.processing = True  # 设置标志位，开始处理
                # 获取本地温度
                temperature = self.get_temperature()
                if temperature is not None:
                    # 构建发送给 AgentProcess 的消息
                    temperature_info = f"当前温度: {temperature:.2f} 度"

                    # 调用 set_llm_content 服务发送温度信息（异步）
                    self.send_temperature_to_llm(temperature_info)
                else:
                    self.get_logger().warn("无法读取温度，跳过发送到 LLM。")
                    self.processing = False  # 清除标志位
            else:
                # 如果 'action' 字段不存在或不包含 "get_temperature()", 则视为响应消息
                if response:
                    # 仅发布 response 字段的内容到 tts_node/tts_text
                    self.publish_tts(response)
                else:
                    self.get_logger().warn("LLM 响应为空，未发布 TTS 消息。")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"解析 JSON 失败: {e}")
        except Exception as e:
            self.get_logger().error(f"处理 LLM 返回结果时发生错误: {e}")

    def clean_json_message(self, message):
        """
        清理接收到的 JSON 消息，去除 ```json 和 ``` 标记。
        """
        try:
            # 使用正则表达式去除 ```json 和 ```
            pattern = r"```json\s*(.*?)\s*```"
            match = re.search(pattern, message, re.DOTALL)
            if match:
                json_str = match.group(1).strip()
                self.get_logger().info(f"清理后的 JSON 数据: {json_str}")
                return json_str
            else:
                # 如果没有代码块标记，尝试直接返回原始消息
                self.get_logger().warn("消息中未找到 ```json 标记，尝试直接解析。")
                return message.strip()
        except Exception as e:
            self.get_logger().error(f"清理 JSON 消息时出错: {e}")
            return None

    def send_temperature_to_llm(self, temperature_info):
        """
        异步调用 /agent_process/set_llm_content 服务将温度信息发送给 LLM。
        """
        # 创建服务请求
        request = SetString.Request()
        request.data = temperature_info

        self.get_logger().info(f"调用 /agent_process/set_llm_content 服务发送温度信息: {request.data}")

        # 调用服务
        future = self.set_llm_content_client.call_async(request)
        future.add_done_callback(self.set_llm_content_callback)

    def set_llm_content_callback(self, future):
        """
        处理 /agent_process/set_llm_content 服务的响应。
        """
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"收到 LLM 响应: {response.message}")

                # 添加日志，查看 response.message 的内容
                self.get_logger().info(f"LLM 响应的原始 message: {response.message}")

                # 清理 response.message 中的 ```json 和 ```
                cleaned_message = self.clean_json_message(response.message)
                if not cleaned_message:
                    self.get_logger().warn("清理后的 message 为空，无法解析。")
                    return

                # 尝试解析 cleaned_message 为 JSON
                try:
                    response_data = json.loads(cleaned_message)
                    self.get_logger().info(f"解析后的 JSON 数据: {response_data}")

                    # 提取 'response' 字段
                    response_field = response_data.get("response", "")
                    self.get_logger().info(f"提取的 'response' 字段: {response_field}")

                    if response_field:
                        self.publish_tts(response_field)
                    else:
                        self.get_logger().warn("LLM 响应中未找到 'response' 字段。")
                except json.JSONDecodeError:
                    # 如果 cleaned_message 不是 JSON，尝试使用正则表达式提取 'response' 字段
                    self.get_logger().info("LLM 响应不是 JSON，尝试使用正则表达式提取 'response' 字段。")
                    match = re.search(r'"response"\s*:\s*"([^"]+)"', cleaned_message)
                    if match:
                        response_field = match.group(1)
                        self.get_logger().info(f"使用正则表达式提取的 'response' 字段: {response_field}")
                        self.publish_tts(response_field)
                    else:
                        self.get_logger().info("无法提取 'response' 字段，直接使用 message 字段。")
                        self.publish_tts(cleaned_message)
            else:
                self.get_logger().error("LLM 处理失败。")
        except Exception as e:
            self.get_logger().error(f"调用 /agent_process/set_llm_content 服务时出错: {e}")
        finally:
            self.processing = False  # 清除标志位

    def publish_tts(self, message):
        """
        发布 TTS 消息到 tts_node/tts_text 主题。
        """
        self.get_logger().info(f"准备发布 TTS 消息: {message}")
        tts_msg = String()
        tts_msg.data = message
        self.tts_text_pub.publish(tts_msg)
        self.get_logger().info(f"发布 TTS 消息: {tts_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMControlServo('llm_control_servo')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LLMControlServo 节点已关闭')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
