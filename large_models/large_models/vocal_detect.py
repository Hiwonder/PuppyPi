#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger, Empty

from speech import speech
from large_models.config import *

class VocalDetect(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.running = True
        self.enable_wakeup = True
        
        
        
        self.declare_parameter('awake_method', 'wonderecho') 
        # self.declare_parameter('mic_type', 'mic6_circle') 
        # self.declare_parameter('port', '/dev/ring_mic')  
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
            # self.kws = speech.CircleMic(port, awake_word, mic_type, enable_setting) 
            self.kws = speech.KWS('py310', wakeup_model_path, sensitivity=0.5, apply_frontend=False) 
        elif self.awake_method == 'wonderecho':
            self.wonderecho = speech.WonderEcho('/dev/ttyUSB0')
        else:
            self.kws = speech.KWS('py310', wakeup_model_path, sensitivity=0.5, apply_frontend=False)
       
        
        voiced_confidence = 0.1
        silence_seconds_begin = 3
        record_seconds = 20
        silence_ratio = 0.8

        self.vad = speech.Vad(voiced_confidence, silence_seconds_begin, record_seconds, silence_ratio)        
        self.asr = speech.ASR(asr_api_key, asr_secret_key, asr_cuid) 
        
        self.asr_pub = self.create_publisher(String, '~/asr_result', 1)
        self.create_service(SetBool, '~/enable_wakeup', self.enable_wakeup_srv)
        self.cli = self.create_client(Empty,'/puppy_control/go_home')

        threading.Thread(target=self.pub_callback, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def pub_callback(self):
        if self.enable_wakeup:
            #self.kws.start()
            self.wonderecho.start()
        while self.running:
            if self.enable_wakeup:
                if self.awake_method == 'wonderecho':
                    result = self.wonderecho.detect()
                else:
                    result = self.kws.detect()

                if result:
                    if self.awake_method == 'wonderecho':
                        self.wonderecho.stop()# 停止检测
                    else:
                        self.kws.stop()  # 停止检测

                    speech.play_audio(wakeup_audio_path)  # 唤醒播放
                    asr_msg = String()
                    asr_msg.data = ''
                    if self.vad.voice_recording(recording_audio_path, self.get_logger()): # 开启录音
                        self.get_logger().info('\033[1;32m%s\033[0m' % 'asr...')
                        asr_result = self.asr.asr(recording_audio_path) # 识别录音
                        if asr_result:
                            speech.play_audio(dong_audio_path) 
                                                      
                            asr_msg.data = asr_result
                            self.asr_pub.publish(asr_msg)
                            self.enable_wakeup = False
                            self.get_logger().info('\033[1;32m%s\033[0m' % 'publish asr result:' + asr_result)
                        else:
                            self.get_logger().info('\033[1;32m%s\033[0m' % 'asr none')
                            speech.play_audio(dong_audio_path)
                            speech.play_audio(no_voice_audio_path)                            
                    else:
                        self.get_logger().info('\033[1;32m%s\033[0m' % 'no voice detect')
                        speech.play_audio(dong_audio_path)
                        speech.play_audio(no_voice_audio_path)
                        #self.wonderecho.start()
                        #self.kws.start()
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
