#!/usr/bin/env python
# encoding: utf-8
# @Author: liang
# @Date: 2024/12/18

import cv2
import sys
import queue
import rospy
import threading
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, Empty
from cv_bridge import CvBridge
from speech import speech
from config import lingyi_api_key, lingyi_base_url, start_audio_path
from large_models.srv import SetString, SetStringRequest
from puppy_control.srv import SetRunActionName
from action import PuppyControlNode

sys.path.append('/home/ubuntu/software/puppypi_control/')
from action_group_control import runActionGroup, stopActionGroup


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
        rospy.wait_for_service('agent_process/set_model')
        self.cli = rospy.ServiceProxy('/puppy_control/go_home', Empty)

        # 初始化过程
        self.init_process()

    def get_node_state(self, req):
        resp = Trigger.Response()
        resp.success = True
        return resp

    def init_process(self):
        """初始化过程"""
        # 设置模型
        msg_model = SetStringRequest()
        msg_model.data = 'vllm'
        try:
            rospy.loginfo("正在调用 set_model_client 服务...")
            self.set_model_client(msg_model)
            rospy.loginfo("设置模型成功")
        except rospy.ServiceException as e:
            rospy.logerr("设置模型失败: %s" % e)

        # 播放启动音频
        try:
            speech.play_audio(start_audio_path)
            rospy.loginfo("播放启动音频")
        except Exception as e:
            rospy.logerr("播放启动音频失败: %s" % e)

        # 启动图像处理线程
        threading.Thread(target=self.process, daemon=True).start()
        rospy.loginfo('启动成功')

        # 创建初始化完成服务
        rospy.Service('~/init_finish', Trigger, self.get_node_state)

    def vllm_result_callback(self, msg):
        """处理来自 VLLM 的结果回调"""
        self.vllm_result = msg.data  # 仅存储接收到的数据
        rospy.loginfo("接收到 VLLM 结果回调: %s" % self.vllm_result)
        self.handle_vllm_result()  # 调用处理函数

    def handle_vllm_result(self):
        """直接播放VLLM结果"""
        if not self.vllm_result:
            rospy.logwarn("VLLM 结果为空，跳过处理")
            return

        try:
            # 直接播放接收到的文本
            response_msg = String()
            response_msg.data = self.vllm_result
            self.tts_text_pub.publish(response_msg)
            rospy.loginfo("发布 TTS 消息: %s" % self.vllm_result)

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
