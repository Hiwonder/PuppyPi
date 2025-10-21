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
import sys
from sdk import common
from action import PuppyControlNode  
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge,CvBridgeError
import subprocess
from speech import speech
from large_models.config import *
#from track_anything import ObjectTracker
from large_models_msgs.srv import SetString
from puppy_control_msgs.srv import SetRunActionName
sys.path.append('/home/ubuntu/software/puppypi_control/') 
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup
from puppy_kinematics import HiwonderPuppy, PWMServoParams
PROMPT = '''
你作为智能机器人助手，可以识别出图像中的物品颜色，并根据我的描述找到目标物体的位置。response的回答要生动有趣
如果识别到"红色"，object:"red", "color": "red"
如果没有识别到“红色”，则需要说明面前没有该颜色物品 object:"red", "color": "None"，
只需要返回检测到的目标物体的名称以及动作执行结果。

例如，如果我的指令是：
"夹取面前的蓝色方块"
你输出：
{
 "type": "detect",
 "object": "blue",
 "color": "None",
 "response": "面前没有蓝色方块"
}
只回复json本身即可，不要回复其它内容。
'''
class VLLMTrack(Node):
    def __init__(self, name):
        rclpy.init()
        self.bridge = CvBridge()
        super().__init__(name)
        #self.fps = fps.FPS()  # 帧率统计器
        self.image_queue = queue.Queue(maxsize=2)
        self.action = []
        self.vllm_result = ''
        self.running = True
        #self.track = ObjectTracker()
        self.vllm = speech.YiVLLM(lingyi_api_key, lingyi_base_url)
        self.puppy_control_node = PuppyControlNode()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)
        #self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_callback, 1)
        self.run_action_group_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup')
        # self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        # self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetString, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_vllm_prompt_client = self.create_client(SetString, 'agent_process/set_vllm_prompt')
        self.set_vllm_prompt_client.wait_for_service()

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

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
            
    def vllm_result_callback(self, msg):
    	#runActionGroup('stand_with_arm.d6a', True)
        self.vllm_result = msg.data
        self.result = eval(self.vllm_result[self.vllm_result.find('{'):self.vllm_result.find('}') + 1])

        if 'response' in self.result:
            response = self.result['response']
        else:
            response = self.result
        # 检查 object 和 color 字段
        if 'object' in self.result and 'color' in self.result:
            if self.result['object'] == self.result['color']:
                print(1)  # 打印 1
                runActionGroup('Clamping.d6a', True) 
                time.sleep(0.5)
                setServoPulse(9, 1200, 300)
                runActionGroup('place1.d6a', True) 
                time.sleep(0.3)
                runActionGroup('look_down.d6a', True) 
            else:
                print(2)  # 打印 2

        response_msg = String()
        response_msg.data = response
        self.tts_text_pub.publish(response_msg)
        
        self.get_logger().info(response)


    # def vllm_result_callback(self, msg):
    #     self.vllm_result = msg.data
    #     if 'action' in self.vllm_result:
    #         self.result = eval(self.vllm_result[self.vllm_result.find('{'):self.vllm_result.find('}') + 1])
    #         if 'action' in self.result:
    #             action_list = self.result['action']
    #         if 'response' in self.result:
    #             response = self.result['response']
    #         else:
    #             response = self.result
            
    #         # 检查 object 字段并调用对应方法
    #         if 'object' in self.result and 'color' in self.result:
    #                 if self.result['object'] == self.result['color']:
    #                     print(1) 
    #                     runActionGroup('Clamping.d6a', True) 
    #                     time.sleep(0.5)
    #                     setServoPulse(9, 1200, 300)
    #                     runActionGroup('place1.d6a', True) 
    #                     time.sleep(0.3)
    #                     runActionGroup('look_down.d6a', True) 
    #                 else:
    #                     print(2)  

    def process(self):
        while self.running:
            try:
                image = self.image_queue.get(timeout=1)
            except queue.Empty:
                continue

            if self.vllm_result != '':
                try:
                    self.vllm_result = eval(self.vllm_result[self.vllm_result.find('{'):self.vllm_result.find('}') + 1])
                    self.get_logger().info('VLLM Result: %s' % str(self.vllm_result))
                except Exception as e:
                    self.get_logger().error(f"Failed to process result: {e}")
                #msg = SetBool.Request()
                #msg.data = True
                #self.send_request(self.awake_client, msg)
                self.vllm_result = ''
            top_left = (190, 270)  # 矩形框左上角坐标
            bottom_right = (450, 480)  # 矩形框右下角坐标
            color = (0, 255, 255)  # 矩形框颜色 (黄色)
            thickness = 2  # 矩形框厚度
            cv2.rectangle(image, top_left, bottom_right, color, thickness)

            # 显示图像
            cv2.imshow('vllm_color_track', image)
            cv2.waitKey(1)

        cv2.destroyAllWindows()



    def play_audio_callback(self, msg):
        if msg.data:
            req = SetBool.Request()
            req.data = True
            self.send_request(self.awake_client, req)
            
    def image_callback(self, ros_image):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)
    # def image_callback(self, ros_image):
    #     try:
    #         # 将 ROS Image 消息转换为 OpenCV 图像
    #         rgb_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
    #     except CvBridgeError as e:
    #         self.get_logger().error(f"Failed to convert image: {e}")
    #         return

    #     # 在图像上绘制矩形框
    #     top_left = (190, 270)
    #     bottom_right = (450, 480)
    #     color = (0, 255, 255)  # 黄色
    #     thickness = 2
    #     cv2.rectangle(rgb_image, top_left, bottom_right, color, thickness)

    #     if self.image_queue.full():
    #         self.image_queue.get()
    #     self.image_queue.put(rgb_image)

def main():
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
