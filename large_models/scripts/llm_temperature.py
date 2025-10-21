#!/usr/bin/env python
# encoding: utf-8
# @Author: liang
# @Date: 2024/12/03

import smbus
import time
import rospy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from large_models.srv import SetString, SetLLM
from puppy_control.srv import SetRunActionName
import json
import re
from speech import speech  # 假设speech模块在ROS1环境中可用

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

class LLMControlServo:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('llm_control_servo', anonymous=False)

        # 初始化温度传感器
        self.aht10 = AHT10()

        # 创建发布者
        self.tts_text_pub = rospy.Publisher('tts_node/tts_text', String, queue_size=10)

        # 创建订阅者
        self.subscription = rospy.Subscriber('/agent_process/result', String, self.llm_result_callback)

        # 创建服务客户端
        rospy.wait_for_service('/puppy_control/go_home')
        self.go_home_cli = rospy.ServiceProxy('/puppy_control/go_home', Empty)

        rospy.wait_for_service('/agent_process/set_llm')
        self.set_llm_client = rospy.ServiceProxy('/agent_process/set_llm', SetLLM)

        rospy.wait_for_service('/agent_process/set_llm_content')
        self.set_llm_content_client = rospy.ServiceProxy('/agent_process/set_llm_content', SetString)

        # 配置 LLM
        self.configure_llm(app_id="9a4f9740-bb98-4738-8146-b0502c641d96",
                          conversation_id="675f7fd7-3e8b-447f-bc8b-807978713489")

        rospy.loginfo('LLMControlServo 节点已启动，等待消息...')

        # 标志位，防止同时处理多个请求
        self.processing = False

    def configure_llm(self, app_id, conversation_id):
        """
        调用 /agent_process/set_llm 服务配置 LLM。
        """
        rospy.loginfo("配置 LLM: app_id={}, conversation_id={}".format(app_id, conversation_id))
        try:
            # 正确的服务调用方式，传递关键字参数
            response = self.set_llm_client(app_id=app_id, conversation_id=conversation_id)
            if response.success:
                rospy.loginfo("LLM 配置成功: {}".format(response.message))
            else:
                rospy.logerr("LLM 配置失败: {}".format(response.message))
        except rospy.ServiceException as e:
            rospy.logerr("调用 /agent_process/set_llm 服务时出错: {}".format(e))

        # 播放启动音频
        start_audio_path = "/home/ubuntu/puppypi/src/large_models/scripts/resources/audio/start_audio.wav"
        speech.play_audio(start_audio_path,volume=60)

        # 调用 go_home 服务
        try:
            self.go_home_cli()
        except rospy.ServiceException as e:
            rospy.logerr("调用 /puppy_control/go_home 服务时出错: {}".format(e))

    def get_temperature(self):
        """
        获取当前温度并返回。
        """
        try:
            temperature = self.aht10.getData()
            rospy.loginfo("当前温度: {:.2f} °C".format(temperature))
            return temperature
        except Exception as e:
            rospy.logerr("温度传感器读取失败: {}".format(e))
            return None

    def llm_result_callback(self, msg):
        """
        处理从 /agent_process/result 接收到的结果。
        """
        rospy.loginfo("收到 /agent_process/result 消息: {}".format(msg.data))

        try:
            # 去除代码块标记 ```json 和 ```
            cleaned_data = self.clean_json_message(msg.data)
            if not cleaned_data:
                rospy.logerr("清理后的数据为空，无法解析。")
                return

            # 解析 JSON 数据
            result = json.loads(cleaned_data)
            if not isinstance(result, dict):
                rospy.logerr("接收到的 JSON 数据不是一个对象。")
                return

            action_list = result.get('action', [])
            response = result.get('response', '')

            # 如果 'action' 字段存在且包含 "get_temperature()", 则是请求消息
            if "get_temperature()" in action_list:
                if self.processing:
                    rospy.logwarn("当前正在处理一个请求，忽略新的请求。")
                    return

                self.processing = True  # 设置标志位，开始处理
                # 获取本地温度
                temperature = self.get_temperature()
                if temperature is not None:
                    # 构建发送给 AgentProcess 的消息
                    temperature_info = "当前温度: {:.2f} °C".format(temperature)
                    # 调用 set_llm_content 服务发送温度信息
                    self.send_temperature_to_llm(temperature_info)
                else:
                    rospy.logwarn("无法读取温度，跳过发送到 LLM。")
                    self.processing = False  # 清除标志位
            else:
                # 如果 'action' 字段不存在或不包含 "get_temperature()", 则视为响应消息
                if response:
                    # 仅发布 response 字段的内容到 tts_node/tts_text
                    self.publish_tts(response)
                else:
                    rospy.logwarn("LLM 响应为空，未发布 TTS 消息。")

        except json.JSONDecodeError as e:
            rospy.logerr("解析 JSON 失败: {}".format(e))
        except Exception as e:
            rospy.logerr("处理 LLM 返回结果时发生错误: {}".format(e))

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
                rospy.loginfo("清理后的 JSON 数据: {}".format(json_str))
                return json_str
            else:
                # 如果没有代码块标记，尝试直接返回原始消息
                rospy.logwarn("消息中未找到 ```json 标记，尝试直接解析。")
                return message.strip()
        except Exception as e:
            rospy.logerr("清理 JSON 消息时出错: {}".format(e))
            return None

    def send_temperature_to_llm(self, temperature_info):
        """
        调用 /agent_process/set_llm_content 服务将温度信息发送给 LLM。
        """
        try:
            # 正确的服务调用方式，传递关键字参数
            response = self.set_llm_content_client(data=temperature_info)
            if response.success:
                rospy.loginfo("收到 LLM 响应: {}".format(response.message))
                self.process_llm_response(response.message)
            else:
                rospy.logerr("LLM 处理失败。")
        except rospy.ServiceException as e:
            rospy.logerr("调用 /agent_process/set_llm_content 服务时出错: {}".format(e))
        finally:
            self.processing = False  # 清除标志位

    def process_llm_response(self, message):
        """
        处理LLM的响应消息。
        """
        # 清理 response.message 中的 ```json 和 ```
        cleaned_message = self.clean_json_message(message)
        if not cleaned_message:
            rospy.logwarn("清理后的 message 为空，无法解析。")
            return

        # 尝试解析 cleaned_message 为 JSON
        try:
            response_data = json.loads(cleaned_message)
            rospy.loginfo("解析后的 JSON 数据: {}".format(response_data))

            # 提取 'response' 字段
            response_field = response_data.get("response", "")
            rospy.loginfo("提取的 'response' 字段: {}".format(response_field))

            if response_field:
                self.publish_tts(response_field)
            else:
                rospy.logwarn("LLM 响应中未找到 'response' 字段。")
        except json.JSONDecodeError:
            # 如果 cleaned_message 不是 JSON，尝试使用正则表达式提取 'response' 字段
            rospy.loginfo("LLM 响应不是 JSON，尝试使用正则表达式提取 'response' 字段。")
            match = re.search(r'"response"\s*:\s*"([^"]+)"', cleaned_message)
            if match:
                response_field = match.group(1)
                rospy.loginfo("使用正则表达式提取的 'response' 字段: {}".format(response_field))
                self.publish_tts(response_field)
            else:
                rospy.loginfo("无法提取 'response' 字段，直接使用 message 字段。")
                self.publish_tts(cleaned_message)

    def publish_tts(self, message):
        """
        发布 TTS 消息到 tts_node/tts_text 主题。
        """
        rospy.loginfo("准备发布 TTS 消息: {}".format(message))
        tts_msg = String()
        tts_msg.data = message
        self.tts_text_pub.publish(tts_msg)
        rospy.loginfo("发布 TTS 消息: {}".format(tts_msg.data))

def main():
    try:
        node = LLMControlServo()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('LLMControlServo 节点已关闭')

if __name__ == "__main__":
    main()
