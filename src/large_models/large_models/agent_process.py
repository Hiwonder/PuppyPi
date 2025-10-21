#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import queue
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetLLM

class AgentProcess(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.declare_parameter('camera_topic', 'image_raw')
        camera_topic = self.get_parameter('camera_topic').value

        self.prompt = ''
        self.model = ''
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.llm = speech.LLM(llm_app_id, llm_conversation_id, llm_token)
        self.yilm = speech.YiLM(lingyi_api_key, lingyi_base_url)
        
        self.result_pub = self.create_publisher(String, '~/result', 1)
        self.create_subscription(String, 'vocal_detect/asr_result', self.asr_callback, 1)
        self.create_subscription(Image, camera_topic, self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.create_service(SetString, '~/set_model', self.set_model_srv)
        self.create_service(SetLLM, '~/set_llm', self.set_llm_srv)
        self.create_service(SetString, '~/set_llm_content', self.set_llm_content_srv)
        self.create_service(SetString, '~/set_vllm_prompt', self.set_vllm_prompt_srv)
        self.create_service(SetString, '~/set_vllm_content', self.set_vllm_content_srv)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def asr_callback(self, msg):
        # self.get_logger().info(msg.data)
        # 将识别结果传给智能体让他来回答
        if msg.data != '':
            self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
            if self.model == 'vllm':
                image = self.image_queue.get(block=True)
                res = self.yilm.vllm(msg.data, image, prompt=self.prompt)

                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish vllm result:' + str(res))
                msg = String()
                msg.data = res
                self.result_pub.publish(msg)
            else:
                res = self.llm.llm(msg.data)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish llm result:' + str(res))
                msg = String()
                msg.data = res
                self.result_pub.publish(msg)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'asr result none')

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            #self.get_logger().info("Queue is full, removing oldest image.")
            self.image_queue.get()  # Remove oldest image
        self.image_queue.put(bgr_image)
        #self.get_logger().info(f"Image added to queue, queue size: {self.image_queue.qsize()}")


    def set_model_srv(self, request, response):
        # 设置调用哪个模型 vllm/llm
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set model')
        self.get_logger().info("vllm set model ")
        self.model = request.data
        response.success = True
        return response

    def set_vllm_prompt_srv(self, request, response):
        # 设置视觉大模型的prompt
        self.get_logger().info("'vllm prompt")
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set vllm prompt')
        self.prompt = request.data
        response.success = True
        return response

    def set_vllm_content_srv(self, request, response):
        # 输入提示词和文本，视觉智能体返回回答
        self.get_logger().info("response")
        image = self.image_queue.get(block=True) 
        res = self.yilm.vllm(request.query, image, prompt=request.prompt)
        
        response.message = res
        response.success = True
        return response

    def set_llm_srv(self, request, response):
        # 设置调用哪个智能体
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set llm')
        self.llm = speech.LLM(request.app_id, request.conversation_id, llm_token)  
        response.success = True
        return response

    def set_llm_content_srv(self, request, response):
        # 输入文本传给智能体让他来回答
        self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
        res = self.llm.llm(request.data)
        response.message = res
        response.success = True
        return response

def main():
    node = AgentProcess('agent_process')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown() 

if __name__ == "__main__":
    main()
