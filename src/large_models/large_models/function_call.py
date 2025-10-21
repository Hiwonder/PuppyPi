#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import time
import math
import rclpy
import threading
from speech import speech
from puppy_control_msgs.srv import SetRunActionName
from std_srvs.srv import Trigger, Empty, SetBool
from puppy_control_msgs.msg import Velocity, Pose, Gait 
from rclpy.node import Node
from std_msgs.msg import String
from large_models_msgs.srv import SetString

from large_models.config import *

class FunctionCall(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        
        self.action = []
        self.llm_result = ''
        self.running = True
        self.result = ''
        
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.pose_publisher = self.create_publisher(Pose, 'puppy_control/pose', 10)
        self.gait_publisher = self.create_publisher(Gait, 'puppy_control/gait', 10)
        self.velocity_publisher = self.create_publisher(Velocity, 'puppy_control/velocity', 10)
        self.gait_publisher = self.create_publisher(Gait, '/puppy_control/gait', 10)
        
        self.set_gait()
        
        
        self.cli = self.create_client(Empty,'puppy_control/go_home')
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.create_subscription(String, '/agent_process/result', self.llm_result_callback, 1)
        self.run_action_group_srv = self.create_client(SetRunActionName, 'puppy_control/runActionGroup')
     
        # kick_ball client
        self.enter_client_kick_ball = self.create_client(Trigger, 'kick_ball_demo/enter')
        self.enter_client_kick_ball.wait_for_service()
        
        self.start_client_kick_ball = self.create_client(SetBool, 'kick_ball_demo/enable_running')
        self.start_client_kick_ball.wait_for_service()

        self.set_target_client_kick_ball = self.create_client(SetString, 'kick_ball_demo/set_color_target')
        self.set_target_client_kick_ball.wait_for_service()
        
        
        # visual_patorl client
        self.enter_client_visual_patrol = self.create_client(Trigger, 'visual_patrol_demo/enter')
        self.enter_client_visual_patrol.wait_for_service()
        
        self.start_client_visual_patrol = self.create_client(SetBool, 'visual_patrol_demo/enable_running')
        self.start_client_visual_patrol.wait_for_service()

        self.set_target_client_visual_patrol = self.create_client(SetString, 'visual_patrol_demo/set_color_target')
        self.set_target_client_visual_patrol.wait_for_service()
        
        self.init_process()
    
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')      

    def init_process(self):
        self.cli.call_async(Empty.Request())
        speech.play_audio(start_audio_path)      

    def get_node_state(self, request, response):
        response.success = True
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)        
        while rclpy.ok():
            if future.done() and future.result():         
                return future.result()
   
    def llm_result_callback(self, msg):
        self.get_logger().info(msg.data)       
        self.llm_result = msg.data
    
    
    def process(self):
        while self.running:
            if self.llm_result != '':
                self.cli.call_async(Empty.Request())
                if 'action' in self.llm_result:
                    self.result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}') + 1])
                    if 'action' in self.result:
                        action_list = self.result['action']
                    if 'response' in self.result:
                        response = self.result['response']
                    else:
                        response = self.result
                else:
                    time.sleep(0.02)
                response_msg = String()
                response_msg.data = response
                self.tts_text_pub.publish(response_msg)
                
                self.get_logger().info(response)
                self.get_logger().info(str(action_list))
                
                for a in action_list:  
                    if a == "forward":
                        self.set_move(x=15.0)
                        time.sleep(1.05)                        
                        self.set_move(x=0.0)
                    elif a == "back":
                        self.set_move(x=-5.0)
                        time.sleep(1.05)
                        self.set_move(x=0.0)
                    elif a == "turn_left":
                        self.set_move(x=-5.0, yaw_rate=math.radians(30))
                        time.sleep(1.05)
                        self.set_move(x=0.0, yaw_rate=math.radians(0))
                    elif a == "turn_right":
                        self.set_move(x=-5.0, yaw_rate=math.radians(-30))
                        time.sleep(1.05)
                        self.set_move(x=0.0, yaw_rate=math.radians(0))
                    elif a.startswith("kick_ball("):
                        color = a.split("'")[1]                            
                        self.send_request(self.enter_client_kick_ball, Trigger.Request())                                          
                        msg = SetString.Request()
                        msg.data = color
                        self.send_request(self.set_target_client_kick_ball, msg)
                        
                        msg = SetBool.Request()
                        msg.data = True
                        self.send_request(self.start_client_kick_ball, msg)                        
                    elif a.startswith("visual_patrol("):
                        color = a.split("'")[1]                        
                        self.send_request(self.enter_client_visual_patrol, Trigger.Request())
                        
                        msg = SetString.Request()
                        msg.data = color
                        self.send_request(self.set_target_client_visual_patrol, msg)
                        
                        msg = SetBool.Request()
                        msg.data = True
                        self.send_request(self.start_client_visual_patrol, msg)                 
                    else: 
                        time.sleep(0.05)  
                        msg = SetRunActionName.Request()
                        msg.name = f'{a}.d6ac'
                        msg.wait = True
                        result = self.send_request(self.run_action_group_srv, msg)
                    #self.set_move(x=0.0)
                    #time.sleep(0.02)
                    
                
                # After executing the action, wait for the next instruction 
                self.llm_result = ''
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)
            else: 
                time.sleep(0.01)
                


    def set_move(self, x=0.00, y=0.0, yaw_rate=0.0):
        velocity_msg = Velocity(x=x, y=y, yaw_rate=yaw_rate)
        self.velocity_publisher.publish(velocity_msg)
        
    def set_gait(self,overlap_time = 0.15, swing_time = 0.2, clearance_time = 0.0, z_clearance = 5.0):
        # overlap_time:4脚全部着地的时间，单位秒(the time when all four legs touch the ground, measured in seconds)
        # swing_time：2脚离地时间，单位秒(the time duration when legs are off the ground, measured in second)
        # clearance_time：前后脚间隔时间，单位秒(the time interval between the front and rear legs, measured in seconds)
        # z_clearance：走路时，脚抬高的距离，单位cm(the distance the paw needs to be raised during walking, measured in centimeters)

        self.gait_publisher.publish(Gait(overlap_time = overlap_time, swing_time = swing_time, clearance_time =clearance_time, z_clearance = z_clearance))

def main():
    node = FunctionCall('function_call')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown() 

if __name__ == "__main__":
    main()
