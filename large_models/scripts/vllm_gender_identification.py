#!/usr/bin/env python
# encoding: utf-8
# @Author: liang
# @Date: 2024/12/18

import rospy
import cv2
import math
import json
import time
import threading
import queue
import subprocess
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from large_models.srv import SetString, SetStringRequest
from puppy_control.srv import SetRunActionName, SetRunActionNameRequest
from speech import speech
from config import *
from action import PuppyControlNode

PROMPT = '''
你作为智能机器人助手，可以识别出图像中的内容，并根据我的描述识别是男生还是女生。
输出要求：
- 返回一个 JSON 对象，包含以下字段：
  - "type": 始终为 "detect"
  - "object": 识别到的目标物体名称，若未识别到则为 "None"
  - "action": 键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]
  - "response": 根据识别到目标是man还是women进行回复
-如果指令包含动作需求，需根据指令提供对应的 `action`（按执行顺序排列的函数名称字符串数组）
-如果指令包含多个动作需求，需根据指令提供对应的 `action`，并确保所有指定的动作都被包含在 `action` 列表中，且顺序与指令中动作的顺序一致。函数名称只能从以下列表中选择：
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
* 识别到男生的叫一下:man()
* 识别到女生的叫一下:woman()
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
"你面前的男生做两个俯卧撑，然后叫一下"
你输出：
{
 "type": "detect",
 "object": "man",
 "action": ["push_up()", "push_up()", "man()"],
 "response": "面前是个男孩子喔，要我去咬他吗"
}
例如，如果我的指令是：
"你面前的是男生还是女生"
你输出：
{
 "type": "detect",
 "object": "women",
 "action": ["woman()"],
 "response": "是女孩子要我去打个招呼吗"
}
只回复json本身即可，不要回复其它内容。不能乱做动作需要根据提示做动作，叫一声也是动作，也是要执行的，并且response是一定要返回输出，不管是那个response都需要返回
'''

class VLLMTrack(object):
    def __init__(self, name):
        rospy.init_node(name)
        self.action_done_event = threading.Event()
        self.image_queue = queue.Queue(maxsize=2)
        self.bridge = CvBridge()
        self.action = []
        self.vllm_result = ''
        self.running = True
        self.yilm = speech.YiLM(lingyi_api_key, lingyi_base_url)
        self.puppy_control_node = PuppyControlNode()
        
        # 发布者
        self.tts_text_pub = rospy.Publisher('tts_node/tts_text', String, queue_size=1)
        
        # 订阅者
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber('agent_process/result', String, self.vllm_result_callback)
        
        # 服务客户端
        self.run_action_group_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
        self.awake_client = rospy.ServiceProxy('vocal_detect/enable_wakeup', SetBool)
        self.set_model_client = rospy.ServiceProxy('agent_process/set_model', SetString)
        self.set_vllm_prompt_client = rospy.ServiceProxy('agent_process/set_vllm_prompt', SetString)
        self.go_home_client = rospy.ServiceProxy('/puppy_control/go_home', Empty)
        
        # 等待服务可用
        rospy.loginfo("等待服务可用...")
        self.awake_client.wait_for_service()
        self.set_model_client.wait_for_service()
        self.set_vllm_prompt_client.wait_for_service()
        
        # 初始化过程
        self.init_process()
    
    def get_node_state(self, req):
        return Trigger.Response(success=True, message="初始化完成")
    
    def init_process(self):
        # 设置 VLLM 提示
        prompt_msg = SetStringRequest()
        prompt_msg.data = PROMPT
        try:
            self.set_vllm_prompt_client(prompt_msg)
            rospy.loginfo("已设置 VLLM 提示。")
        except rospy.ServiceException as e:
            rospy.logerr("设置 VLLM 提示失败: %s" % e)
        
        # 设置模型
        model_msg = SetStringRequest()
        model_msg.data = 'vllm'
        try:
            self.set_model_client(model_msg)
            rospy.loginfo("已设置模型为 vllm。")
        except rospy.ServiceException as e:
            rospy.logerr("设置模型失败: %s" % e)
        
        # 播放开始音频
        speech.play_audio(start_audio_path)
        
        # 启动处理线程
        threading.Thread(target=self.process, daemon=True).start()
        
        # 创建初始化完成的服务
        rospy.Service('~/init_finish', Trigger, self.get_node_state)
        rospy.loginfo('\033[1;32m%s\033[0m' % '节点已启动')
        
        # 调用回家服务
        try:
            self.go_home_client()
            rospy.loginfo("已调用回家服务。")
        except rospy.ServiceException as e:
            rospy.logerr("调用回家服务失败: %s" % e)
    
    def vllm_result_callback(self, msg):
        """回调函数，接收 VLLM 结果并触发处理逻辑"""
        self.vllm_result = msg.data
        rospy.loginfo("接收到 VLLM 结果回调: %s" % self.vllm_result)
        self.handle_vllm_result()
    
    def handle_vllm_result(self):
        """处理 VLLM 结果，包括解析动作和发布响应"""
        if not self.vllm_result:
            rospy.logwarn("VLLM 结果为空，跳过处理。")
            return  # 如果没有结果，直接返回
        
        try:
            # 提取 JSON 字符串并解析
            start_idx = self.vllm_result.find('{')
            end_idx = self.vllm_result.rfind('}') + 1
            if start_idx == -1 or end_idx <= start_idx:
                raise ValueError("无效的 JSON 格式")
            result_json = self.vllm_result[start_idx:end_idx]
            rospy.loginfo("解析出的 JSON: %s" % result_json)
            result = json.loads(result_json)  
            
            action_list = result.get('action', [])
            response = result.get('response', '')
            
            rospy.loginfo("动作列表: %s" % action_list)
            rospy.loginfo("响应: %s" % response)
            
            # 发布中文响应消息
            self.publish_response(response)
            
            # 等待6秒
            rospy.loginfo("等待6秒后执行动作...")
            time.sleep(6)
            
            # 执行动作
            self.execute_actions(action_list)
        
        except json.JSONDecodeError as e:
            rospy.logerr("JSON 解码错误: %s" % e)
        except Exception as e:
            rospy.logerr("处理 VLLM 结果时出错: %s" % e)
        finally:
            self.vllm_result = ''  # 清空结果，避免重复处理
    
    def execute_actions(self, action_list):
        """执行动作列表中的动作"""
        rospy.loginfo("开始执行动作列表...")
        for action in action_list:
            if action and action != "None" and isinstance(action, str):
                try:
                    # 去掉 '()'，例如 'push_up()' -> 'push_up'
                    action_name = action.strip('()')
                    rospy.loginfo("准备执行动作: %s" % action_name)
                    
                    # 动态执行动作函数
                    func = getattr(self.puppy_control_node, action_name, None)
                    if func:
                        func()
                        rospy.loginfo("成功执行动作: %s" % action_name)
                    else:
                        rospy.logwarn("动作函数 '%s' 未找到。" % action_name)
                except Exception as e:
                    rospy.logerr("执行动作 '%s' 失败: %s" % (action, e))
            else:
                rospy.logwarn("无效的动作: %s，跳过执行。" % action)
        rospy.loginfo("动作列表执行完毕。")
    
    def publish_response(self, response):
        """发布中文响应消息"""
        if response:
            response_msg = String()
            response_msg.data = response
            rospy.loginfo("发布响应消息: %s" % response)
            time.sleep(1)
            self.tts_text_pub.publish(response_msg)
        else:
            rospy.logwarn("响应消息为空，未发布任何内容。")
    
    def process(self):
        """处理图像队列并显示图像"""
        while not rospy.is_shutdown() and self.running:
            try:
                image = self.image_queue.get(block=True)
                
                # 如果有 VLLM 结果，暂存处理
                if self.vllm_result != '':
                    msg = String()
                    msg.data = self.vllm_result
                    # self.tts_text_pub.publish(msg)  # 可选的 TTS 发布
                    self.vllm_result = ''
                
                cv2.imshow('image', image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    rospy.loginfo("检测到 'q' 按键，关闭图像窗口。")
                    break
            except Exception as e:
                rospy.logerr("处理图像时出错: %s" % e)
        cv2.destroyAllWindows()
        rospy.loginfo("图像处理线程已结束。")
    
    def image_callback(self, ros_image):
        """图像回调函数，将 ROS Image 转换为 OpenCV 图像并放入队列"""
        try:
            # 使用 cv_bridge 将 ROS Image 转换为 OpenCV 图像
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            
            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像
                discarded_image = self.image_queue.get()
            # 将图像放入队列
            self.image_queue.put(rgb_image)
        except CvBridgeError as e:
            rospy.logerr("转换图像时出错: %s" % e)
        except Exception as e:
            rospy.logerr("处理图像时出错: %s" % e)

def main():
    node = None
    try:
        node = VLLMTrack('vllm_gender_identification')
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('节点已关闭')
    except Exception as e:
        rospy.logerr("程序运行时出错: %s" % e)
    finally:
        if node and node.running:
            node.running = False
        rospy.loginfo("程序结束。")

if __name__ == "__main__":
    main()
