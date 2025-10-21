#!/usr/bin/env python3
# encoding: utf-8
# @Author: liang
# @Date: 2024/11/18

import rospy
import rosnode 
import threading
import time
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from speech import speech
from config import *
from large_models.srv import SetInt32, SetInt32Response

# 定义日志包装器类
class ROS1Logger:
    def info(self, msg):
        rospy.loginfo(msg)

class VocalDetect(object):
    def __init__(self, name):
        # 初始化ROS节点
        rospy.init_node(name, anonymous=False)

        self.running = True

        # 获取参数，如果没有设置则使用默认值
        self.awake_method = rospy.get_param('~awake_method', 'xf')
        self.mic_type = rospy.get_param('~mic_type', 'mic6_circle')
        self.port = rospy.get_param('~port', '/dev/ring_mic')
        self.awake_word = rospy.get_param('~awake_word', 'xiao3 huan4 xiao3 huan4')
        self.enable_setting = rospy.get_param('~enable_setting', False)
        self.enable_wakeup = rospy.get_param('~enable_wakeup', False)
        self.mode = rospy.get_param('~mode', 1)

        # 根据唤醒方法初始化相应的对象
        if self.awake_method == 'xf':
            # 注释掉环形麦克风相关的部分
            # self.kws = speech.CircleMic(self.port, self.awake_word, self.mic_type, self.enable_setting)
            pass
        else:
            self.kws = speech.WonderEcho('/dev/ttyUSB0')

        # 初始化 VAD 和 ASR
        voiced_confidence = 0.1
        silence_seconds_begin = 3
        record_seconds = 20
        silence_ratio = 0.8

        if self.awake_method == 'xf':
            self.vad = speech.Vad(voiced_confidence, silence_seconds_begin, record_seconds, silence_ratio, channel=1)
        else:
            self.vad = speech.Vad(voiced_confidence, silence_seconds_begin, record_seconds, silence_ratio)
        
        self.asr = speech.ASR(asr_api_key, asr_secret_key, asr_cuid) 
        
        # 创建发布者
        self.asr_pub = rospy.Publisher('~asr_result', String, queue_size=1)
        self.awake_angle_pub = rospy.Publisher('~angle', Int32, queue_size=1)
        
        # 创建服务
        self.set_mode_service = rospy.Service('~set_mode', SetInt32, self.set_mode_srv)
        self.enable_wakeup_service = rospy.Service('~enable_wakeup', SetBool, self.enable_wakeup_srv)
        self.init_finish_service = rospy.Service('~init_finish', Trigger, self.get_node_state)
        
        # 创建日志包装器实例
        self.logger = ROS1Logger()
        
        # 启动线程以处理发布回调
        threading.Thread(target=self.pub_callback, daemon=True).start()
        
        rospy.loginfo('\033[1;32m%s\033[0m' % 'VocalDetect 启动成功')

    def get_node_state(self, request):
        # 服务回调，返回节点状态
        response = TriggerResponse()
        response.success = True
        response.message = "VocalDetect 正在运行中。"
        return response

    def record(self, mode, angle=None):
        # 开启录音并进行ASR识别
        if self.vad.voice_recording(recording_audio_path, self.logger):  # 开启录音
            rospy.loginfo('\033[1;32m%s\033[0m' % 'asr...')
            asr_result = self.asr.asr(recording_audio_path)  # 识别录音
            if asr_result:
                speech.play_audio(dong_audio_path)
                if self.awake_method == 'xf' and self.mode == 1:
                    msg = Int32()
                    msg.data = int(angle)
                    self.awake_angle_pub.publish(msg)
                asr_msg = String()
                asr_msg.data = asr_result
                self.asr_pub.publish(asr_msg)
                self.enable_wakeup = False
                rospy.loginfo('\033[1;32m%s\033[0m' % ('publish asr result:' + asr_result))
            else:
                rospy.loginfo('\033[1;32m%s\033[0m' % 'asr none')
                speech.play_audio(dong_audio_path)
                if mode != 3:
                    speech.play_audio(no_voice_audio_path)
        else:
            rospy.loginfo('\033[1;32m%s\033[0m' % 'no voice detect')
            speech.play_audio(dong_audio_path)
            if mode != 3:
                speech.play_audio(no_voice_audio_path)

    def pub_callback(self):
        # 唤醒检测的主循环
        if self.enable_wakeup and self.mode == 1:
            # 注释掉环形麦克风相关的启动
            # self.kws.start()
            pass
        while self.running:
            if self.enable_wakeup:
                if self.mode == 1:
                    result = self.kws.detect()
                    if result:
                        self.kws.stop()  # 停止检测
                        speech.play_audio(wakeup_audio_path)  # 唤醒播放
                        self.record(self.mode, result)
                    else:
                        rospy.sleep(0.02)
                elif self.mode == 2:
                    self.record(self.mode)
                    self.mode = 0
                elif self.mode == 3:
                    self.record(self.mode)
                else:
                    rospy.sleep(0.02)
            else:
                rospy.sleep(0.02)
        rospy.signal_shutdown('Stopping VocalDetect')

    def enable_wakeup_srv(self, request):
        """
        服务回调，启用或禁用唤醒检测
        """
        rospy.loginfo('\033[1;32m%s\033[0m' % 'enable_wakeup')
        if request.data:
            # 如果启用唤醒检测，启动相应的检测方法
            if self.awake_method == 'xf':
                # 注释掉环形麦克风相关的启动
                # self.kws.start()
                pass
            else:
                self.kws.start()
        self.enable_wakeup = request.data
        
        return SetBoolResponse(success=True, message="Wakeup state set to: " + str(self.enable_wakeup))

    def set_mode_srv(self, request):
        """
        服务回调，设置模式
        """
        rospy.loginfo('\033[1;32m%s\033[0m' % 'set_mode')
        self.mode = int(request.data)
        if self.mode == 2:
            self.enable_wakeup = True
        return SetInt32Response(success=True)


def main():
    node = VocalDetect('vocal_detect')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('节点已关闭')


if __name__ == "__main__":
    main()
