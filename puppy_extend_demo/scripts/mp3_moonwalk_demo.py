#!/usr/bin/env python3
# coding=utf8
import os
import math
import rospy
import signal
import subprocess
from std_msgs.msg import *
from puppy_control.srv import SetRunActionName
from puppy_control.msg import Velocity, Pose, Gait

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

PuppyPose = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000, 'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
GaitConfig = {'overlap_time': 0.3, 'swing_time': 0.2, 'clearance_time': 0.0, 'z_clearance': 5}

MP3_DIR = "//home/ubuntu/puppypi/src/puppy_extend_demo/scripts/MP3"
mpg123_process = None  


def get_mp3_files(directory):
    """从目录中读取所有 MP3 文件"""
    files = [f for f in os.listdir(directory) if f.endswith(".mp3")]
    return {i + 1: os.path.join(directory, f) for i, f in enumerate(files)}


def play_audio(file_path):

    global mpg123_process
    try:
        if not os.path.exists(file_path):
            print(f"文件不存在: {file_path}")
            return
        mpg123_process = subprocess.Popen(["mpg123", file_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"正在播放音乐: {file_path}")
    except FileNotFoundError:
        print("未找到 `mpg123` 命令，检查系统安装了 `mpg123`。")


def stop_audio():
    """终止音乐播放"""
    global mpg123_process
    if mpg123_process:
        mpg123_process.terminate()
        mpg123_process.wait()
        mpg123_process = None
        print("音乐播放已停止")


def stop_all():
    """终止所有操作：音乐和动作"""
    print("正在关闭程序...")
    stop_audio()  # 停止音乐
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)  # 停止机器人动作
    runActionGroup_srv('stand.d6ac', True)  # 让机器人站立停止
    rospy.signal_shutdown("程序已安全退出")  # 关闭 ROS 节点


def signal_handler(signum, frame):
    """捕获 Ctrl+C 信号并终止程序"""
    stop_all()


def linkage(times=1):
    """示例：多轴联动"""
    for i in range(0, 15, 1):
        PuppyPose.update({'roll': math.radians(i), 'pitch': math.radians(0)})
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                             height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=30)
        rospy.sleep(0.03)


if __name__ == "__main__":
    # 注册 Ctrl+C 信号处理
    signal.signal(signal.SIGINT, signal_handler)


    rospy.init_node('mp3_moonwalk_demo', disable_signals=True)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
    rospy.sleep(0.3)

    mp3_files = get_mp3_files(MP3_DIR)
    if not mp3_files:
        print(f"目录 {MP3_DIR} 中没有找到 MP3 文件，请检查目录内容。")
        exit()

    print("可用歌曲列表:")
    for num, path in mp3_files.items():
        print(f"{num}: {os.path.basename(path)}")

    try:
        song_number = int(input("请输入要播放的歌曲编号: "))
        if song_number in mp3_files:
            play_audio(mp3_files[song_number])  
        else:
            print("输入的编号无效，请重试。")
            exit()
    except ValueError:
        print("输入无效，请输入数字编号。")
        exit()

    try:
        # 机器狗站立
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                             height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)
        rospy.sleep(0.5)
        PuppyGaitConfigPub.publish(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
                                   clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance'])

        # 原地踏步
        PuppyVelocityPub.publish(x=0.01, y=0, yaw_rate=0)
        rospy.sleep(3)
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
        rospy.sleep(1)

        # 多轴联动
        linkage(2)

        # 向前走
        PuppyVelocityPub.publish(x=5, y=0, yaw_rate=0)
        rospy.sleep(3)
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)

        # 向后走
        rospy.sleep(1)
        PuppyVelocityPub.publish(x=-5, y=0, yaw_rate=0)
        rospy.sleep(3)
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)

        # 滑步动作组
        runActionGroup_srv('moonwalk.d6ac', True)
        rospy.sleep(0.5)

    finally:
        stop_all() 
