import os
import sys
import rospy
import math
import serial
from std_msgs.msg import *
from puppy_control.msg import Velocity, Pose, Gait

print('''
**********************************************************
******************功能:语音交互例程(function: voice interaction routine)*************************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(press Ctrl+C to close this program, please try multiple times if fail)
----------------------------------------------------------
''')

# 初始化机器狗的姿态和步态配置
PuppyPose = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000, 'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
GaitConfig = {'overlap_time': 0.2, 'swing_time': 0.2, 'clearance_time': 0.0, 'z_clearance': 3}

# 停止函数
run_st = True
def Stop():
    global run_st
    run_st = False
    print('关闭中...')

# 解析串口数据
def parse_serial_data(data):
    # 将字节数据转换为十六进制字符串
    hex_data = ' '.join(format(byte, '02X') for byte in data)
    print(f"Received data: {hex_data}")

    # 根据不同的指令执行相应的动作
    if hex_data == "AA 55 00 76 FB":  # 原地踏步
        print("执行原地踏步")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
        rospy.sleep(0.5)
        PuppyVelocityPub.publish(x=0.1, y=0, yaw_rate=0)
        rospy.sleep(2)
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)

    elif hex_data == "AA 55 00 0A FB":  # 立正
        print("执行立正")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    elif hex_data == "AA 55 00 0B FB":  # 趴下
        print("执行趴下")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-6, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)

    elif hex_data == "AA 55 00 8D FB":  # 抬头
        print("执行抬头")
        PuppyPose = {'roll':math.radians(0), 'pitch':math.radians(20), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)

    elif hex_data == "AA 55 00 09 FB":  # 停止
        print("停止识别")
        PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
        global run_st
        run_st = False

if __name__ == "__main__":
    # 初始化 ROS 节点
    rospy.init_node('voice_interaction_demo')
    rospy.on_shutdown(Stop)

    # 发布器初始化
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)

    rospy.sleep(0.5)

    # 机器狗站立
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                         height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500)
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
                               clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance'])

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    while run_st:
        # 读取所有串口数据并解析
        if ser.in_waiting > 0:
            data = ser.read_all()  # 读取所有可用数据
            parse_serial_data(data)  # 解析数据
        rospy.sleep(0.1)
