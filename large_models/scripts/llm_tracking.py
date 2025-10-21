#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18

import re
import time
import rospy
import threading
import json

from config import *
from speech import speech
from std_msgs.msg import String
from std_srvs.srv import (
    Trigger, TriggerRequest, TriggerResponse,
    SetBool, SetBoolRequest, SetBoolResponse
)

from large_models.srv import (
    SetLLM, SetLLMRequest, SetLLMResponse,
    SetColor, SetColorRequest, SetColorResponse
)

class LLMColorSorting:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('llm_tracking', anonymous=False)

        # 初始化变量
        self.action = []
        self.llm_result = ''
        self.running = True

        # 发布者：发布到 'tts_node/tts_text' 话题
        self.tts_text_pub = rospy.Publisher('tts_node/tts_text', String, queue_size=10)

        # 订阅者：订阅 '/agent_process/result' 话题
        rospy.Subscriber('/agent_process/result', String, self.llm_result_callback)

        # 服务客户端：'agent_process/set_llm'
        self.set_llm_client = rospy.ServiceProxy('agent_process/set_llm', SetLLM)
        rospy.loginfo("等待服务 'agent_process/set_llm' 可用...")
        rospy.wait_for_service('agent_process/set_llm')
        rospy.loginfo("服务 'agent_process/set_llm' 已连接。")

        # 服务客户端：'/color_tracking/enter'
        self.enter_client = rospy.ServiceProxy('/color_tracking/enter', Trigger)
        rospy.loginfo("等待服务 '/color_tracking/enter' 可用...")
        rospy.wait_for_service('/color_tracking/enter')
        rospy.loginfo("服务 '/color_tracking/enter' 已连接。")

        # 服务客户端：'/color_tracking/enable_color_tracking'
        self.start_client = rospy.ServiceProxy('/color_tracking/enable_color_tracking', SetBool)
        rospy.loginfo("等待服务 '/color_tracking/enable_color_tracking' 可用...")
        rospy.wait_for_service('/color_tracking/enable_color_tracking')
        rospy.loginfo("服务 '/color_tracking/enable_color_tracking' 已连接。")

        # 服务客户端：'/color_tracking/set_color'
        self.set_color_client = rospy.ServiceProxy('/color_tracking/set_color', SetColor)
        rospy.loginfo("等待服务 '/color_tracking/set_color' 可用...")
        rospy.wait_for_service('/color_tracking/set_color')
        rospy.loginfo("服务 '/color_tracking/set_color' 已连接。")

        # 服务客户端：'/color_tracking/stop'
        self.stop_client = rospy.ServiceProxy('/color_tracking/stop', Trigger)
        rospy.loginfo("等待服务 '/color_tracking/stop' 可用...")
        rospy.wait_for_service('/color_tracking/stop')
        rospy.loginfo("服务 '/color_tracking/stop' 已连接。")

        # 初始化线程锁
        self.llm_result_lock = threading.Lock()

        # 创建一次性定时器，用于初始化过程
        # ROS1 中不支持 oneshot 参数，移除它并保留定时器引用
        self.init_timer = rospy.Timer(rospy.Duration(0.1), self.init_process)

        # 创建服务：'~/init_finish'
        self.init_finish_service = rospy.Service('~/init_finish', Trigger, self.get_node_state)

        rospy.loginfo('\033[1;32m%s\033[0m' % '节点已启动，等待初始化完成...')

        # 启动处理线程
        self.process_thread = threading.Thread(target=self.process)
        self.process_thread.daemon = True
        self.process_thread.start()

    def get_node_state(self, request):
        response = TriggerResponse()
        response.success = True
        response.message = "节点运行正常。"
        return response

    def init_process(self, event):
        # 取消定时器
        self.init_timer.shutdown()

        # 调用 SetLLM 服务
        try:
            set_llm_request = SetLLMRequest()
            set_llm_request.app_id = '04c230e9-2267-4831-b861-4a77f6f9be68'
            set_llm_request.conversation_id = 'c99865f6-00e7-4103-b381-dd2ea9fc59e7'
            set_llm_response = self.set_llm_client(set_llm_request)
            rospy.loginfo("调用 SetLLM 服务成功。")
        except rospy.ServiceException as e:
            rospy.logerr("调用 SetLLM 服务失败: %s" % e)

        # 调用 /color_tracking/enter 服务
        try:
            enter_request = TriggerRequest()
            enter_response = self.enter_client(enter_request)
            if enter_response.success:
                rospy.loginfo("已进入颜色跟踪模式。")
            else:
                rospy.logwarn("进入颜色跟踪模式失败: %s" % enter_response.message)
        except rospy.ServiceException as e:
            rospy.logerr("调用 /color_tracking/enter 服务失败: %s" % e)

        # 播放音频
        if 'start_audio_path' in globals():
            speech.play_audio(start_audio_path)
        else:
            rospy.logerr("变量 'start_audio_path' 未定义。")

        rospy.loginfo('\033[1;32m%s\033[0m' % '初始化过程完成，节点开始运行。')

    def send_request(self, client, msg):
        try:
            response = client(msg)
            rospy.loginfo("服务调用成功。")
            return response
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s" % e)
            return None

    def llm_result_callback(self, msg):
        with self.llm_result_lock:
            self.llm_result += msg.data  # 追加接收到的数据
        rospy.loginfo("接收到数据: %s" % msg.data)

    def process(self):
        buffer = ""
        while not rospy.is_shutdown() and self.running:
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
                            rospy.logerr("JSON 解码失败: %s" % e)
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
            rospy.logerr("结果格式无效：缺少 'action' 或 'action' 不是列表。")
            return

        if 'grab' in result['action']:
            if 'object' in result and result['object']:
                color = result['object']
                rospy.loginfo("动作: grab, 颜色: %s" % color)

                # 调用 SetColor 服务设置颜色
                try:
                    set_color_request = SetColorRequest()
                    set_color_request.color = color  # 确保 SetColorRequest 中有 'color' 字段
                    set_color_response = self.set_color_client(set_color_request)
                    if set_color_response.success:
                        rospy.loginfo("设置颜色为 %s 成功。" % color)
                    else:
                        rospy.logerr("设置颜色为 %s 失败: %s" % (color, set_color_response.message))
                except rospy.ServiceException as e:
                    rospy.logerr("调用 SetColor 服务失败: %s" % e)

                # 调用 SetBool 服务启用颜色跟踪
                try:
                    set_bool_request = SetBoolRequest()
                    set_bool_request.data = True
                    set_bool_response = self.start_client(set_bool_request)
                    if set_bool_response.success:
                        rospy.loginfo("颜色跟踪已启用。")
                    else:
                        rospy.logerr("启用颜色跟踪失败: %s" % set_bool_response.message)
                except rospy.ServiceException as e:
                    rospy.logerr("调用 SetBool 服务失败: %s" % e)

                # 发布 TTS 消息
                if 'response' in result and result['response']:
                    tts_msg = String()
                    tts_msg.data = result['response']
                    self.tts_text_pub.publish(tts_msg)
                    rospy.loginfo("发布 TTS 消息: %s" % tts_msg.data)
            else:
                rospy.logerr("收到 'grab' 动作但缺少 'object' 或 'object' 为空。")

        elif 'stop' in result['action']:
            if 'object' in result and not result['object']:
                rospy.loginfo("动作: stop")

                # 调用 SetBool 服务禁用颜色跟踪
                try:
                    set_bool_request = SetBoolRequest()
                    set_bool_request.data = False
                    set_bool_response = self.start_client(set_bool_request)
                    if set_bool_response.success:
                        rospy.loginfo("颜色跟踪已禁用。")
                    else:
                        rospy.logerr("禁用颜色跟踪失败: %s" % set_bool_response.message)
                except rospy.ServiceException as e:
                    rospy.logerr("调用 SetBool 服务失败: %s" % e)

                # 调用 Trigger 服务停止颜色跟踪
                try:
                    stop_request = TriggerRequest()
                    stop_response = self.stop_client(stop_request)
                    if stop_response.success:
                        rospy.loginfo("颜色跟踪已停止。")
                    else:
                        rospy.logerr("停止颜色跟踪失败: %s" % stop_response.message)
                except rospy.ServiceException as e:
                    rospy.logerr("调用 Trigger 服务失败: %s" % e)

                # 发布 TTS 消息
                if 'response' in result and result['response']:
                    tts_msg = String()
                    tts_msg.data = result['response']
                    self.tts_text_pub.publish(tts_msg)
                    rospy.loginfo("发布 TTS 消息: %s" % tts_msg.data)
            else:
                rospy.logerr("收到 'stop' 动作但 'object' 不为空。")

        elif not result['action'] and 'response' in result:
            # 处理空动作，发布 TTS 消息
            tts_msg = String()
            tts_msg.data = result['response']
            self.tts_text_pub.publish(tts_msg)
            rospy.loginfo("发布 TTS 消息: %s" % tts_msg.data)

        else:
            rospy.logwarn("收到未处理的 'action'。")

    def shutdown(self):
        self.running = False
        rospy.loginfo("节点正在关闭...")

if __name__ == "__main__":
    try:
        node = LLMColorSorting()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
