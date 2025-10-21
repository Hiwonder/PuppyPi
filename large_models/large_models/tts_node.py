#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import time
import rclpy
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger,SetBool
from std_msgs.msg import String, Bool

from speech import speech
from large_models.config import *

class TTSNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.text = None
        self.tts = speech.TTS(tts_api_key, tts_secret_key, tts_cuid)
       
        self.play_finish_pub = self.create_publisher(Bool, '~/play_finish', 1)
        self.create_subscription(String, '~/tts_text', self.tts_callback, 1)
       
        threading.Thread(target=self.tts_process, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def tts_callback(self, msg):
        # self.get_logger().info(msg.data)
        self.text = msg.data

    def tts_process(self):
        while True:
            if self.text is not None:
                if self.text == '':
                    speech.play_audio(no_voice_audio_path)
                else:
                    self.tts.tts(self.text, file_path=tts_audio_path)
                self.text = None
                msg = Bool()
                msg.data = True
                self.play_finish_pub.publish(msg)
            else:
                time.sleep(0.01)

def main():
    node = TTSNode('tts_node')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown() 

if __name__ == "__main__":
    main()
