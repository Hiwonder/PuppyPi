 #!/usr/bin/env python3
# coding=utf8
# 第7课 MP3模块实验(Lesson 7 MP3 Module)
import math
import rclpy
import time
import signal
import os
import subprocess
from speech import speech
from rclpy.node import Node
from puppy_control_msgs.srv import SetRunActionName
from puppy_control_msgs.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************功能:MP3模块例程(function: MP3 module routine)*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close the program, please try multiple times if fail)
----------------------------------------------------------
''')


class Mp3MoonwalkDemo(Node):
    def __init__(self):
        super().__init__('mp3_moonwalk_demo')
        signal.signal(signal.SIGINT, self.stop) 
        
        self.MP3_DIR = "./MP3"
               
        # 创建发布者和客户端    
        self.pose_publisher = self.create_publisher(Pose, '/puppy_control/pose',10)
        self.gait_publisher = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.velocity_publisher = self.create_publisher(Velocity, '/puppy_control/velocity',10)
        self.run_action_group_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup',)
        
        self.get_mp3_files()
        
        # 初始化机器狗姿态和步态
        self.set_pose()
        self.set_gait()

        # 执行动作   
        self.timer = self.create_timer(0.1, self.mp3_moon_callback)  
           
    def get_mp3_files(self):
        """从目录中读取所有 MP3 文件"""
        files = [f for f in os.listdir(self.MP3_DIR) if f.endswith(".mp3")]
        self.mp3_files = {i + 1: os.path.join(self.MP3_DIR, f) for i, f in enumerate(files)}
        if not self.mp3_files:
            self.get_logger().error(f"目录 {self.MP3_DIR} 中没有找到 MP3 文件，请检查目录内容。")
            exit()

        self.get_logger().info("可用歌曲列表:")
        for num, path in self.mp3_files.items():
            self.get_logger().info(f"{num}: {os.path.basename(path)}")
            
    def play_audio(self, file_path):
        """播放音频文件"""
        if not os.path.exists(file_path):
            self.get_logger().error(f"文件不存在: {file_path}")
            return
        self.mpg123_process = subprocess.Popen(["mpg123", file_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.get_logger().info(f"正在播放音乐: {file_path}")
   
   
    def mp3_moon_callback(self):
        # 原地踏步(stepping in place)
        self.set_move(x=0.01) 
        time.sleep(3)
        self.set_move(x=0.0)
        time.sleep(1)
        
        # 多轴联动(multi-axis linkage)
        self.linkage(2) 
        
    
        # 机器狗站立(robot dog stands up)
        self.set_pose()
        
        # 向前走(move forward)
        self.set_move(x=5.0)
        time.sleep(3)      
        # 向后走(walk backward)
        self.set_move(x=-5.0) 
        time.sleep(2)
        # 停止(stop)
        self.set_move(x=0.0) 
        time.sleep(1)
        
        # 执行滑步动作组(perform the sliding step action group)
        msg = SetRunActionName.Request()
        msg.name = 'moonwalk.d6ac'
        msg.wait = True
        self.run_action_group_srv.call_async(msg)
        time.sleep(0.1)
        self.run_action_group_srv.call_async(msg)
        time.sleep(0.1)
        
        # 机器狗站立(robot dog stands up)
        self.set_pose()
        time.sleep(0.5)
        
        # 向前走(move forward)
        self.set_move(x=5.0)
        time.sleep(3)      
        # 向后走(walk backward)
        self.set_move(x=-5.0) 
        time.sleep(2)
        # 停止(stop)
        self.set_move()
        time.sleep(1)
        
        
    def linkage(self, times = 1):        
        # times:循环次数(number of iterations)
        for i in range(0,15,1):
            self.set_pose(roll=math.radians(i),run_time = 30)
            time.sleep(0.03)
        for i in range(0,15,1):
            self.set_pose(pitch=math.radians(i),run_time = 30)
            time.sleep(0.03)
            
        for s in range(times):
            for i in range(15,-15,-1):
                self.set_pose(roll=math.radians(i),pitch=math.radians(15),run_time = 30)
                time.sleep(0.03)
            for i in range(15,-15,-1):
                self.set_pose(roll=math.radians(-15),pitch=math.radians(i),run_time = 30)
                time.sleep(0.03)
                 
            for i in range(-15,15,1):
                self.set_pose(roll=math.radians(i),pitch=math.radians(-15),run_time = 30)
                time.sleep(0.03)
            for i in range(-15,15,1):
                self.set_pose(roll=math.radians(15),pitch=math.radians(i),run_time = 30)
                time.sleep(0.03)
                
    def set_pose(self,roll=math.radians(0), pitch=math.radians(0), yaw=0.000, height=-10.0, x_shift=0.5, stance_x= 0.0, stance_y=0.0,run_time=500):
        # stance_x：4条腿在x轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the X-axis, measured in centimeters)
        # stance_y：4条腿在y轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the Y-axis, measured in centimeters)
        # x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm(the distance traveled by the four legs along the x-axis determines the degree of forward or backward tilt during walking: smaller distances lead to more forward tilt, while larger distances result in more backward tilt. Adjusting the x_shift parameter can help maintain balance during the dog's movement, measured in centimeters)
        # height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm(the height of the dog, measured from the toe to the axis  of rotation of the thigh, is in centimeters)
        # pitch： 狗身体的俯仰角，单位弧度(the pitch angle of the dog's body, measured in radians)  
        self.pose_publisher.publish(Pose(stance_x=stance_x, stance_y=stance_y, x_shift=x_shift,
                                           height=height, roll=roll, pitch=pitch, yaw=yaw, run_time=500))
                                           
    def set_gait(self,overlap_time = 0.3, swing_time = 0.2, clearance_time = 0.0, z_clearance = 5.0):
        # overlap_time:4脚全部着地的时间，单位秒(the time when all four legs touch the ground, measured in seconds)
        # swing_time：2脚离地时间，单位秒(the time duration when legs are off the ground, measured in second)
        # clearance_time：前后脚间隔时间，单位秒(the time interval between the front and rear legs, measured in seconds)
        # z_clearance：走路时，脚抬高的距离，单位cm(the distance the paw needs to be raised during walking, measured in centimeters)

        self.gait_publisher.publish(Gait(overlap_time = overlap_time, swing_time = swing_time, clearance_time =clearance_time, z_clearance = z_clearance))
        
    def set_move(self, x=0.00, y=0.0, yaw_rate=0.0):
        self.velocity_publisher.publish(Velocity(x=x, y=y, yaw_rate=yaw_rate)) 
                    
    def stop(self):
        self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
        self.get_logger().info('Shutting down...')
        
def main(args=None):
    rclpy.init(args=args)
    node = Mp3MoonwalkDemo()
    
    try:
        song_number = int(input("请输入要播放的歌曲编号: "))
        if song_number in node.mp3_files:
            node.play_audio(node.mp3_files[song_number])  
        else:
            node.get_logger().error("输入的编号无效，请重试。")
            exit()
    except ValueError:
        node.get_logger().error("输入无效，请输入数字编号。")
        exit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("intteruprt------------")
        pass

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
 

if __name__ == '__main__':
    main()
