#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetInt32

class VocalDetect(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.running = True

        # 声明参数
        self.declare_parameter('awake_method', 'wonderecho')  # 修改唤醒方法为wonderecho
        # self.declare_parameter('mic_type', 'mic6_circle')  # 注释掉环形麦克风相关参数
        # self.declare_parameter('port', '/dev/ring_mic')  # 注释掉环形麦克风端口
        self.declare_parameter('enable_wakeup', False)
        self.declare_parameter('enable_setting', False)
        self.declare_parameter('awake_word', 'xiao3 huan4 xiao3 huan4')
        self.declare_parameter('mode', 1)

        self.awake_method = self.get_parameter('awake_method').value
        # mic_type = self.get_parameter('mic_type').value  
        # port = self.get_parameter('port').value  
        awake_word = self.get_parameter('awake_word').value
        enable_setting = self.get_parameter('enable_setting').value 
        self.enable_wakeup = self.get_parameter('enable_wakeup').value
        self.mode = int(self.get_parameter('mode').value)

        if self.awake_method == 'xf':
            # self.kws = speech.CircleMic(port, awake_word, mic_type, enable_setting)  # 注释掉环形麦克风初始化部分
            self.kws = speech.KWS('py310', wakeup_model_path, sensitivity=0.5, apply_frontend=False) 
        elif self.awake_method == 'wonderecho':
            self.wonderecho = speech.WonderEcho('/dev/ttyUSB0')  # 使用WonderEcho初始化唤醒方法
        else:
            self.kws = speech.KWS('py310', wakeup_model_path, sensitivity=0.5, apply_frontend=False)

        voiced_confidence = 0.1
        silence_seconds_begin = 3
        record_seconds = 20
        silence_ratio = 0.8

        self.vad = speech.Vad(voiced_confidence, silence_seconds_begin, record_seconds, silence_ratio)

        self.asr = speech.ASR(asr_api_key, asr_secret_key, asr_cuid) 
        
        self.asr_pub = self.create_publisher(String, '~/asr_result', 1)
        self.awake_angle_pub = self.create_publisher(Int32, '~/angle', 1)
        self.create_service(SetInt32, '~/set_mode', self.set_mode_srv)
        self.create_service(SetBool, '~/enable_wakeup', self.enable_wakeup_srv)

        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def record(self, mode, result=None):
        if self.vad.voice_recording(recording_audio_path, self.get_logger()):  # 开启录音
            self.get_logger().info('\033[1;32m%s\033[0m' % 'asr...')
            asr_result = self.asr.asr(recording_audio_path)  # 识别录音
            if asr_result:
                speech.play_audio(dong_audio_path)
                if self.awake_method == 'wonderecho' and self.mode == 1 and result is not None: 
                    msg = Int32()
                    msg.data = int(result)  # 使用传入的 result
                    self.awake_angle_pub.publish(msg)
                asr_msg = String()
                asr_msg.data = asr_result
                self.asr_pub.publish(asr_msg)
                self.enable_wakeup = False
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish asr result:' + asr_result)
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'asr none')
                speech.play_audio(dong_audio_path)
                if mode != 3:
                    speech.play_audio(no_voice_audio_path)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'no voice detect')
            speech.play_audio(dong_audio_path)
            if mode != 3:
                speech.play_audio(no_voice_audio_path)

    def pub_callback(self):
        if self.enable_wakeup and self.mode == 1:
            if self.awake_method == 'wonderecho':
                self.wonderecho.start()
            else:
                self.kws.start()
        while self.running:
            if self.enable_wakeup:
                if self.mode == 1:
                    if self.awake_method == 'wonderecho':
                        result = self.wonderecho.detect()
                    else:
                        result = self.kws.detect()
                    
                    if result:
                        if self.awake_method == 'wonderecho':
                            self.wonderecho.stop()  # 停止检测
                        else:
                            self.kws.stop()  # 停止检测
                        speech.play_audio(wakeup_audio_path)  # 唤醒播放
                        self.record(self.mode, result)  # 传递 result
                    else:
                        time.sleep(0.02)
                elif self.mode == 2:
                    self.record(self.mode)
                    self.mode = 0
                elif self.mode == 3:
                    self.record(self.mode)
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.02)
        rclpy.shutdown()


    def enable_wakeup_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % ('enable_wakeup'))
        if self.awake_method == 'wonderecho':
            self.wonderecho.start()
        else:
            self.kws.start()
        self.enable_wakeup = request.data
        
        response.success = True
        return response 

    def set_mode_srv(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % ('set_mode'))
        self.mode = int(request.data)
        if self.mode == 2:
            self.enable_wakeup = True
        response.success = True
        return response 

def main():
    node = VocalDetect('vocal_detect')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown() 

if __name__ == "__main__":
    main()
