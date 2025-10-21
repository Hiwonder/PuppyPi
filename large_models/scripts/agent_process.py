#!/usr/bin/env python3
# encoding: utf-8
# @Author: liang
# @Date: 2024/11/18

import rospy
import queue
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import Image

from speech import speech
from config import *
from large_models.srv import SetString, SetStringResponse, SetLLM, SetLLMResponse

class AgentProcess(object):
    def __init__(self, name):
        # 初始化ROS节点
        rospy.init_node(name, anonymous=False)

        # 获取参数，如果没有设置则使用默认值
        self.camera_topic = rospy.get_param('~camera_topic', '/usb_cam/image_raw')

        self.prompt = ''
        self.model = ''
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.llm = speech.LLM(llm_app_id, llm_conversation_id, llm_token)
        self.yilm = speech.YiLM(lingyi_api_key, lingyi_base_url)

        # 创建发布者
        self.result_pub = rospy.Publisher('~result', String, queue_size=1)

        # 创建订阅者
        rospy.Subscriber('vocal_detect/asr_result', String, self.asr_callback)
        rospy.Subscriber(self.camera_topic, Image, self.image_callback)

        # 创建服务
        self.set_model_service = rospy.Service('~set_model', SetString, self.set_model_srv)
        self.set_llm_service = rospy.Service('~set_llm', SetLLM, self.set_llm_srv)
        self.set_llm_content_service = rospy.Service('~set_llm_content', SetString, self.set_llm_content_srv)
        self.set_vllm_prompt_service = rospy.Service('~set_vllm_prompt', SetString, self.set_vllm_prompt_srv)
        self.set_vllm_content_service = rospy.Service('~set_vllm_content', SetString, self.set_vllm_content_srv)
        self.init_finish_service = rospy.Service('~init_finish', Trigger, self.get_node_state)

        rospy.loginfo('\033[1;32m%s\033[0m' % '启动成功')

    def get_node_state(self, request):
        """
        处理 ~init_finish 服务请求，返回节点状态。
        """
        response = TriggerResponse()
        response.success = True
        response.message = "节点初始化完成"
        return response

    def asr_callback(self, msg):
        """
        处理语音识别结果的回调函数。
        """
        # 处理语音识别结果
        if msg.data != '':
            rospy.loginfo('\033[1;32m%s\033[0m' % '思考中...')
            if self.model == 'vllm':
                try:
                    image = self.image_queue.get(block=True, timeout=5)
                except queue.Empty:
                    rospy.logwarn('图像队列为空，无法获取图像')
                    return
                res = self.yilm.vllm(msg.data, image, prompt=self.prompt)
                rospy.loginfo('\033[1;32m%s\033[0m' % ('发布vllm结果:' + str(res)))
                result_msg = String()
                result_msg.data = res
                self.result_pub.publish(result_msg)
            else:
                res = self.llm.llm(msg.data)
                rospy.loginfo('\033[1;32m%s\033[0m' % ('发布llm结果:' + str(res)))
                result_msg = String()
                result_msg.data = res
                self.result_pub.publish(result_msg)
        else:
            rospy.loginfo('\033[1;32m%s\033[0m' % '语音识别结果为空')

    def image_callback(self, ros_image):
        """
        处理摄像头图像的回调函数。
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except Exception as e:
            rospy.logerr('图像转换错误: %s' % str(e))
            return
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                pass
        # 将图像放入队列
        self.image_queue.put(bgr_image)

    def set_model_srv(self, request):
        """
        设置调用的模型类型（vllm/llm）的服务处理函数。
        """
        rospy.loginfo('\033[1;32m%s\033[0m' % '设置模型类型')
        self.model = request.data
        response = SetStringResponse()
        response.success = True
        response.message = "模型类型设置成功"
        return response

    def set_vllm_prompt_srv(self, request):
        """
        设置视觉大模型的提示词的服务处理函数。
        """
        rospy.loginfo('\033[1;32m%s\033[0m' % '设置vllm提示词')
        self.prompt = request.data
        response = SetStringResponse()
        response.success = True
        response.message = "vllm提示词设置成功"
        return response

    def set_vllm_content_srv(self, request):
        """
        输入提示词和文本，视觉智能体返回回答的服务处理函数。
        """
        try:
            image = self.image_queue.get(block=True, timeout=5)
        except queue.Empty:
            rospy.logwarn('图像队列为空，无法获取图像')
            response = SetStringResponse()
            response.success = False
            response.message = '图像队列为空'
            return response
        res = self.yilm.vllm(request.query, image, prompt=request.prompt)
        response = SetStringResponse()
        response.message = res
        response.success = True
        rospy.loginfo('\033[1;32m%s\033[0m' % ('vllm处理结果:' + str(res)))
        return response

    def set_llm_srv(self, request):
        """
        设置调用的LLM智能体的服务处理函数。
        """
        rospy.loginfo('\033[1;32m%s\033[0m' % '设置LLM智能体')
        self.llm = speech.LLM(request.app_id, request.conversation_id, llm_token)
        response = SetLLMResponse()
        response.success = True
        response.message = "LLM智能体设置成功"
        return response

    def set_llm_content_srv(self, request):
        """
        输入文本传给智能体让其回答的服务处理函数。
        """
        rospy.loginfo('\033[1;32m%s\033[0m' % '思考中...')
        res = self.llm.llm(request.data)
        response = SetStringResponse()
        response.message = res
        response.success = True
        rospy.loginfo('\033[1;32m%s\033[0m' % ('llm处理结果:' + str(res)))
        return response

def main():
    try:
        node = AgentProcess('agent_process')
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('节点已关闭')
    except Exception as e:
        rospy.logerr('出现异常: %s' % str(e))

if __name__ == "__main__":
    main()
