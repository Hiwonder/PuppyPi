#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18

import time
import rospy
import rosnode  
import threading
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from std_msgs.msg import String, Bool

from speech import speech  
from config import *       

class TTSNode(object):
    def __init__(self):
        rospy.init_node('tts_node', anonymous=False)
        
        self.text = None
        self.tts = speech.TTS(tts_api_key, tts_secret_key, tts_cuid)

        # 创建发布者
        self.play_finish_pub = rospy.Publisher('~play_finish', Bool, queue_size=10)
        
        # 创建订阅者
        rospy.Subscriber('~tts_text', String, self.tts_callback)
        
        # 创建服务客户端
        rospy.wait_for_service('/vocal_detect/enable_wakeup')  # 等待服务可用
        self.cli = rospy.ServiceProxy('/vocal_detect/enable_wakeup', SetBool)

        # 创建服务
        rospy.Service('~init_finish', Trigger, self.get_node_state)
        
        # 启动线程处理TTS
        self.tts_thread = threading.Thread(target=self.tts_process, daemon=True)
        self.tts_thread.start()
        
        rospy.loginfo('\033[1;32m%s\033[0m' % 'TTSNode 启动成功')

    def get_node_state(self, request):
        """
        处理 '~init_finish' 服务请求，返回节点状态。
        """
        response = TriggerResponse()
        response.success = True
        response.message = "TTSNode 正在运行中。"
        return response

    def tts_callback(self, msg):
        """
        订阅 '~tts_text' 主题的回调函数。
        更新待处理的文本。
        """
        rospy.logdebug("收到 TTS 文本: %s", msg.data)
        self.text = msg.data

    def tts_process(self):
        """
        持续监控并处理TTS请求。
        将文本转换为语音并发布 'play_finish' 消息。
        同时，调用 '/vocal_detect/enable_wakeup' 服务。
        """
        rate = rospy.Rate(100)  # 100 Hz
        while not rospy.is_shutdown():
            if self.text is not None:
                if self.text.strip() == '':
                    # 如果文本为空，播放无声音频
                    speech.play_audio(no_voice_audio_path)
                else:
                    rospy.logdebug("正在处理TTS文本: %s", self.text)
                    self.tts.tts(self.text, file_path=tts_audio_path)
                
                # 重置文本
                self.text = None
                
                # 发布 'play_finish' 消息
                msg = Bool()
                msg.data = True
                self.play_finish_pub.publish(msg)
                rospy.logdebug("已发布 'play_finish' 消息。")
                
                # 调用 '/vocal_detect/enable_wakeup' 服务
                self.call_enable_wakeup(True)
            else:
                time.sleep(0.01)
            rate.sleep()

    def call_enable_wakeup(self, switch):
        """
        调用 '/vocal_detect/enable_wakeup' 服务，启用或禁用唤醒功能。
        """
        try:
            response = self.cli(switch)
            if response.success:
                rospy.loginfo("已成功调用 enable_wakeup 服务，状态: %s" % response.message)
            else:
                rospy.logwarn("调用 enable_wakeup 服务失败，状态: %s" % response.message)
        except rospy.ServiceException as e:
            rospy.logerr("调用 /vocal_detect/enable_wakeup 服务时出错: %s" % str(e))

def main():
    """
    主函数，实例化TTSNode并保持节点运行。
    """
    try:
        tts_node = TTSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('TTSNode 已被中断，正在关闭。')
    except Exception as e:
        rospy.logerr('发生意外错误: %s', str(e))
    finally:
        rospy.loginfo('TTSNode 已关闭。')

if __name__ == "__main__":
    main()
