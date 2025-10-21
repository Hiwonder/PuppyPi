#!/usr/bin/env python
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
import threading
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger, Empty
from cv_bridge import CvBridge
from speech import speech
from config import lingyi_api_key, lingyi_base_url, start_audio_path  
from large_models.srv import SetString, SetStringRequest  
from puppy_control.srv import SetRunActionName
from action import PuppyControlNode

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup

# 定义提示信息
PROMPT = '''
你作为智能机器人助手，可以实时识别图像中的障碍物，并在 `response` 中输出物品名称及相关提示信息。
规则：
1. 如果识别到障碍物：
   - `object` 返回识别出的物品名称。
   - 在 `response` 中提示用户障碍物的位置，并关心提醒注意安全。
   - 如果指令包含多个动作需求，需根据指令提供对应的 `action`，并确保所有指定的动作都被包含在 `action` 列表中，且顺序与指令中动作的顺序一致。函数名称只能从以下列表中选择：
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
      "response": "门口有障碍物喔，我来叫一声提醒你，你千万要注意避开哦"
     }
     ```
     
     如果指令为：  
     "看看有没有什么障碍物，如果有障碍物的话叫一声，然后再做个俯卧撑提醒我"  
     你返回：
     ```json
     {
      "type": "detect",
      "object": "障碍物名称",
      "action": ["man()", "push_up()"],
      "Play": ["detect"],
      "response": "门口有障碍物喔，叫一声提醒你了要注意避开喔，并做了个俯卧撑来提醒你。"
     }
     ```
     
     # 如果指令为：  
     # "看看有没有什么障碍物，如果有障碍物的话叫一声提醒我"  
     # 你返回：
     # ```json
     # {
      # "type": "detect",
      # "object": "障碍物名称",
      # "action": ["man()"],
      # "Play": ["detect"],
      # "response": "门口有障碍物喔，叫一声提醒你了要注意避开喔"
     # }
     # ```

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

4. **确保所有指定的动作都被包含且顺序正确**。如果指令中包含多个动作，`action` 列表中必须按指令中动作的顺序依次排列所有动作。

5. **示例扩展**：
   - 指令包含多个动作：
     - 指令："如果检测到障碍物，请叫一声并站立提醒我。"
       返回：
       ```json
       {
        "type": "detect",
        "object": "障碍物名称",
        "action": ["man()", "stand()"],
        "Play": ["detect"],
        "response": "门口有障碍物喔，叫一声并站立提醒你了要注意避开喔。"
       }
       ```
     - 指令："帮我看看有没有障碍物，如果有的话握手并点头提醒我。"
       返回：
       ```json
       {
        "type": "detect",
        "object": "障碍物名称",
        "action": ["shake_hands()", "nod()"],
        "Play": ["detect"],
        "response": "门口有障碍物喔，握手并点头提醒你了要注意避开喔。"
       }
       ```
     - 指令："检查是否有障碍物，如果有，踢左脚然后撒娇提醒我。"
       返回：
       ```json
       {
        "type": "detect",
        "object": "障碍物名称",
        "action": ["kick_ball_left()", "bow()"],
        "Play": ["detect"],
        "response": "门口有障碍物喔，踢左脚并撒娇提醒你了要注意避开喔。"
       }
       ```

只需要按照以上规则，返回 JSON 格式的内容即可，不要添加其它内容。确保所有指定的动作都被正确执行，并按照指令中的顺序排列。

'''


class VLLMTrack(object):
    def __init__(self, name):
        rospy.init_node(name)
        self.image_queue = queue.Queue(maxsize=2)
        self.bridge = CvBridge()
        self.vllm_result = ''
        self.running = True
        self.yilm = speech.YiLM(lingyi_api_key, lingyi_base_url)
        self.puppy_control_node = PuppyControlNode()
        self.tts_text_pub = rospy.Publisher('tts_node/tts_text', String, queue_size=1)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.Subscriber('agent_process/result', String, self.vllm_result_callback)
        self.run_action_group_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
        self.set_model_client = rospy.ServiceProxy('agent_process/set_model', SetString)
        self.set_vllm_prompt_client = rospy.ServiceProxy('agent_process/set_vllm_prompt', SetString)
        rospy.wait_for_service('agent_process/set_model')
        rospy.wait_for_service('agent_process/set_vllm_prompt')
        self.cli = rospy.ServiceProxy('/puppy_control/go_home', Empty)

        # 初始化过程
        self.init_process()

    def get_node_state(self, req):
        resp = Trigger.Response()
        resp.success = True
        return resp

    def init_process(self):
        # 设置 VLLM 提示
        msg_prompt = SetStringRequest()
        msg_prompt.data = PROMPT
        try:
            rospy.loginfo("正在调用 set_vllm_prompt_client 服务...")
            self.set_vllm_prompt_client(msg_prompt)
            rospy.loginfo("设置 VLLM 提示成功")
        except rospy.ServiceException as e:
            rospy.logerr("设置 VLLM 提示失败: %s" % e)
        
        # 设置模型
        msg_model = SetStringRequest()
        msg_model.data = 'vllm'
        try:
            rospy.loginfo("正在调用 set_model_client 服务...")
            self.set_model_client(msg_model)
            rospy.loginfo("设置模型成功")
        except rospy.ServiceException as e:
            rospy.logerr("设置模型失败: %s" % e)
        
        # 运行初始动作
        try:
            runActionGroup('look_down.d6ac', True)
            rospy.loginfo("运行初始动作: look_down.d6ac")
        except Exception as e:
            rospy.logerr("运行初始动作失败: %s" % e)
        
        # 播放启动音频
        try:
            speech.play_audio(start_audio_path)
            rospy.loginfo("播放启动音频")
        except Exception as e:
            rospy.logerr("播放启动音频失败: %s" % e)
        
        # 启动图像处理线程
        threading.Thread(target=self.process, daemon=True).start()
        rospy.loginfo('\033[1;32m%s\033[0m' % '启动成功')

        # 创建初始化完成服务
        rospy.Service('~/init_finish', Trigger, self.get_node_state)
        
        runActionGroup('look_down.d6ac', True)
        # try:
        #     self.cli()
        # except rospy.ServiceException as e:
        #     rospy.logerr("调用服务失败: %s" % e)

    def vllm_result_callback(self, msg):
        """处理来自 VLLM 的结果回调"""
        self.vllm_result = msg.data  # 仅存储接收到的数据
        rospy.loginfo("接收到 VLLM 结果回调: %s" % self.vllm_result)
        self.handle_vllm_result()  # 调用处理函数

    def handle_vllm_result(self):
        """处理 VLLM 结果的逻辑，包括动作解析和响应发布"""  
        if not self.vllm_result:
            rospy.logwarn("VLLM 结果为空，跳过处理")
            return

        try:
            # 提取 JSON 字符串并解析
            start = self.vllm_result.find('{')
            end = self.vllm_result.rfind('}') + 1
            if start == -1 or end <= start:
                raise ValueError("无效的 JSON 格式")
            result_json = self.vllm_result[start:end]
            result = json.loads(result_json)
            rospy.loginfo("解析后的 JSON: %s" % result_json)

            action_list = result.get('action', [])
            response = result.get('response', '')

            rospy.loginfo("动作列表: %s" % action_list)
            rospy.loginfo("响应: %s" % response)

            # 执行动作
            for action in action_list:
                if action and action != "None":
                    # 去除 '()'，例如 'push_up()' -> 'push_up'
                    action_name = action.strip('()')
                    rospy.loginfo("准备执行动作: %s" % action_name)

                    # 动态执行动作函数
                    action_func = getattr(self.puppy_control_node, action_name, None)
                    if callable(action_func):
                        try:
                            action_func()
                            rospy.loginfo("成功执行动作: %s" % action_name)
                        except Exception as e:
                            rospy.logerr("执行动作 '%s' 失败: %s" % (action_name, e))
                    else:
                        rospy.logwarn("未知的动作函数: %s" % action_name)
                else:
                    rospy.logwarn("无效的动作: %s，跳过执行。" % action)

            # 发布中文响应消息
            time.sleep(6)
            response_msg = String()
            response_msg.data = response
            self.tts_text_pub.publish(response_msg)
            rospy.loginfo("发布 TTS 消息: %s" % response)

            try:
                self.cli()
                rospy.loginfo("调用回到家服务成功")
            except rospy.ServiceException as e:
                rospy.logerr("调用回到家服务失败: %s" % e)

            time.sleep(3)
            try:
                runActionGroup('look_down.d6ac', True)
                rospy.loginfo("恢复初始动作: look_down.d6ac")
            except Exception as e:
                rospy.logerr("恢复初始动作失败: %s" % e)

        except json.JSONDecodeError as e:
            rospy.logerr("JSON 解析错误: %s" % e)
        except Exception as e:
            rospy.logerr("处理 VLLM 结果出错: %s" % e)
        finally:
            self.vllm_result = ''  # 清空结果，避免重复处理

    def process(self):
        """图像处理线程"""
        rospy.loginfo("图像处理线程已启动。")
        while not rospy.is_shutdown() and self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
                
                # 显示图像
                cv2.imshow('image', image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    rospy.loginfo("检测到 'q' 按键，关闭图像窗口。")
                    break
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr("处理图像时出错: %s" % e)
        cv2.destroyAllWindows()
        rospy.loginfo("图像处理线程已结束。")

    def image_callback(self, ros_image):
        """图像回调函数，将 ROS Image 转换为 OpenCV 图像并放入队列"""
        try:
            # 使用 cv_bridge 将 ROS Image 转换为 OpenCV 图像
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            rospy.loginfo("接收到图像帧")

            if self.image_queue.full():
                # 如果队列已满，丢弃最旧的图像
                discarded = self.image_queue.get()
                rospy.logwarn("图像队列已满，丢弃最旧的图像")

            # 将图像放入队列
            self.image_queue.put(rgb_image)
        except Exception as e:
            rospy.logerr("处理图像时出错: %s" % e)

def main():
    try:
        node = VLLMTrack('vllm_track')
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('关闭节点')
    finally:
        if 'node' in locals() and node.running:
            node.running = False
        cv2.destroyAllWindows()
        rospy.loginfo("程序结束。")

if __name__ == "__main__":
    main()
