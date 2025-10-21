#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18

import rospy
import time
import math
import threading
from puppy_control.srv import SetRunActionName
from std_srvs.srv import Trigger, Empty, SetBool
from puppy_control.msg import Velocity, Pose, Gait
from std_msgs.msg import String
from large_models.srv import SetString
from speech import speech
from config import *

class FunctionCall:
    def __init__(self, name):
        # 初始化 ROS 节点
        rospy.init_node(name)

        # 初始化变量
        self.action = []
        self.llm_result = ''
        self.running = True
        self.result = ''

        # 创建话题发布者
        self.tts_text_pub = rospy.Publisher('tts_node/tts_text', String, queue_size=1)
        self.pose_publisher = rospy.Publisher('puppy_control/pose', Pose, queue_size=10)
        self.gait_publisher = rospy.Publisher('puppy_control/gait', Gait, queue_size=10)
        self.velocity_publisher = rospy.Publisher('puppy_control/velocity', Velocity, queue_size=10)
        
        # 设置步态
        self.set_gait()

        # 创建服务代理
        self.cli = rospy.ServiceProxy('/puppy_control/go_home', Empty)

        self.awake_client = rospy.ServiceProxy('vocal_detect/enable_wakeup', SetBool)
        rospy.Subscriber('/agent_process/result', String, self.llm_result_callback)
        self.run_action_group_srv = rospy.ServiceProxy('puppy_control/runActionGroup', SetRunActionName)

        # kick_ball 客户端
        self.enter_client_kick_ball = rospy.ServiceProxy('kick_ball_demo/enter', Trigger)
        self.start_client_kick_ball = rospy.ServiceProxy('kick_ball_demo/enable_running', SetBool)
        self.set_target_client_kick_ball = rospy.ServiceProxy('kick_ball_demo/set_color_target', SetString)

        # visual_patrol 客户端
        self.enter_client_visual_patrol = rospy.ServiceProxy('visual_patrol_demo/enter', Trigger)
        self.start_client_visual_patrol = rospy.ServiceProxy('visual_patrol_demo/enable_running', SetBool)
        self.set_target_client_visual_patrol = rospy.ServiceProxy('visual_patrol_demo/set_color_target', SetString)

        # 初始化进程
        self.init_process()
        
        # 启动处理线程
        threading.Thread(target=self.process, daemon=True).start()
        
        # 创建初始化完成的服务
        rospy.Service('~init_finish', Trigger, self.get_node_state)
        rospy.loginfo('\033[1;32m%s\033[0m' % 'start')

    def init_process(self):
        # 调用回家服务，并播放起始音频
        self.cli()
        speech.play_audio(start_audio_path)

    def get_node_state(self, request):
        # 返回节点状态的服务回调
        response = Trigger()
        response.success = True
        return response


    def llm_result_callback(self, msg):
        # 处理返回的结果消息
        rospy.loginfo(msg.data)
        self.llm_result = msg.data

    def process(self):
        # 主处理循环
        while self.running:
            if self.llm_result != '':
                # 调用回家服务
                #self.cli(Empty())
                
                # 解析 LLM 结果
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
                
                # 发布 TTS 文本
                response_msg = String()
                response_msg.data = response
                self.tts_text_pub.publish(response_msg)

                rospy.loginfo(response)
                rospy.loginfo(str(action_list))

                # 执行动作列表
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
                        # 处理踢球动作
                        color = a.split("'")[1]
                        
                        self.enter_client_kick_ball()
                 
                        self.set_target_client_kick_ball(color)
                                                
                        self.start_client_kick_ball(True)
                                 
                    elif a.startswith("visual_patrol("):
                        # 处理视觉巡逻动作
                        color = a.split("'")[1]               
                        
                        self.enter_client_visual_patrol()
                        
                        self.set_target_client_visual_patrol(color)
                        
                        self.start_client_visual_patrol(True)                        
                      
                    else:
                        # 处理其他动作
                        time.sleep(0.05)
                        self.run_action_group_srv(f'{a}.d6ac',True)
                # 执行完动作后，等待下一个指令 
                self.llm_result = ''
                self.awake_client(True)
                
            else:
                time.sleep(0.01)

    def set_move(self, x=0.00, y=0.0, yaw_rate=0.0):
        # 发布移动指令
        velocity_msg = Velocity(x=x, y=y, yaw_rate=yaw_rate)
        self.velocity_publisher.publish(velocity_msg)

    def set_gait(self, overlap_time=0.15, swing_time=0.2, clearance_time=0.0, z_clearance=5.0):
        # 设置步态参数并发布
        self.gait_publisher.publish(Gait(overlap_time=overlap_time, swing_time=swing_time, clearance_time=clearance_time, z_clearance=z_clearance))

def main():
    # 主函数
    node = FunctionCall('function_call')
    try:
        rospy.spin()  # 持续运行节点
    except rospy.ROSInterruptException:
        print('shutdown')
    finally:
        rospy.signal_shutdown('Node shutdown')  # 清理节点

if __name__ == "__main__":
    main()
