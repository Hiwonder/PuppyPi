#!/usr/bin/python3
# coding=utf8
# Author: Summer
# Email: 997950600@qq.com
import time
import os
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, UInt16, Float32, Float64, Bool, String, Float32MultiArray
from std_srvs.srv import Empty, SetBool
from ros_robot_controller_msgs.msg import BuzzerState
from geometry_msgs.msg import Point32, Polygon, Twist
from puppy_control_msgs.srv import SetRunActionName
from sensor_msgs.msg import Imu, JointState
from puppy_control_msgs.msg import Velocity, Gait, SetServo, Pose
from rclpy.duration import Duration
from sdk.ArmMoveIK import *
sys.path.append('/home/ubuntu/software/puppypi_control')
from servo_controller import setServoPulse, updatePulse
from action_group_control import runActionGroup, stopActionGroup
from puppy_kinematics import HiwonderPuppy, PWMServoParams
ROS_NODE_NAME = 'puppy_control'

with_arm = 0
if with_arm:
    offset = 0.1
else:
    offset = 0

# 定义PuppyPose各个姿态的参数
Stand = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000, 'height': -10.0, 'x_shift': -0.5 + offset, 'stance_x': 0, 'stance_y': 0}
LieDown = {'roll': 0.000, 'pitch': 0.000, 'yaw': 0.000, 'height': -5.0, 'x_shift': 2.0, 'stance_x': 0, 'stance_y': 0}
LookDown = {'roll': math.radians(0), 'pitch': math.radians(-15.0), 'yaw': 0.000, 'height': -10, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
LookDown_10deg = {'roll': math.radians(0), 'pitch': math.radians(-10.0), 'yaw': 0.000, 'height': -9, 'x_shift': -0.1, 'stance_x': 0, 'stance_y': 0}
LookDown_20deg = {'roll': math.radians(0), 'pitch': math.radians(-20.0), 'yaw': 0.000, 'height': -9, 'x_shift': -0.1, 'stance_x': 0, 'stance_y': 0}
LookDown_30deg = {'roll': math.radians(0), 'pitch': math.radians(-30.0), 'yaw': 0.000, 'height': -9.6, 'x_shift': -1.4, 'stance_x': 1, 'stance_y': 0}

StandLow = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.000, 'height': -7.0, 'x_shift': -0.5, 'stance_x': 0, 'stance_y': 0}
PuppyPose = Stand.copy()  # 初始姿态设为蹲姿

# 定义GaitConfig各个配置的参数
GaitConfigFast = {'overlap_time': 0.1, 'swing_time': 0.15, 'clearance_time': 0.0, 'z_clearance': 5.0}
GaitConfigSlow = {'overlap_time': 0.4, 'swing_time': 0.3, 'clearance_time': 0.26, 'z_clearance': 4.0}
GaitConfigMarkTime = {'overlap_time': 0.2, 'swing_time': 0.1, 'clearance_time': 0.0, 'z_clearance': 5.0}
GaitConfig = GaitConfigFast.copy()

class MPU6050:

    def __init__(self, node):
        self.node = node
        self.subscription = self.node.create_subscription(Imu, '/ros_robot_controller/imu_raw', self.GetImuFun, 10)
        self.data = {'accel': [0, 0, 0], 'gyro': [0, 0, 0]}
        self.SecondOrderFilterX = self._SecondOrderFilter()
        self.SecondOrderFilterY = self._SecondOrderFilter()

    def GetImuFun(self, msg):
        self.data['accel'][0] = msg.linear_acceleration.x
        self.data['accel'][1] = msg.linear_acceleration.y
        self.data['accel'][2] = msg.linear_acceleration.z
        self.data['gyro'][0] = msg.angular_velocity.x
        self.data['gyro'][1] = msg.angular_velocity.y
        self.data['gyro'][2] = msg.angular_velocity.z

    def _SecondOrderFilter(self):
        x1 = 0
        x2 = 0
        y1 = 0
        angle = 0
        K2 = 0.02

        def fun(angle_m, gyro_m, dt=0.01):
            nonlocal x1, x2, y1, angle, K2
            x1 = (angle_m - angle) * (1 - K2) * (1 - K2)
            y1 = y1 + x1 * dt
            x2 = y1 + 2 * (1 - K2) * (angle_m - angle) + gyro_m
            angle = angle + x2 * dt
            return angle

        return fun

    def get_euler_angle(self, dt=0.01):
        data = self.data
        accel_Y = math.atan2(data['accel'][0], data['accel'][2]) * 180 / math.pi
        gyro_Y = data['gyro'][1]
        angleY = self.SecondOrderFilterY(-accel_Y, gyro_Y, dt)

        accel_X = math.atan2(data['accel'][1], data['accel'][2]) * 180 / math.pi
        gyro_X = data['gyro'][0]
        angleX = self.SecondOrderFilterX(accel_X, gyro_X, dt)

        return {'pitch': -math.radians(angleX), 'roll': -math.radians(angleY), 'yaw': 0}

class PUPPY(Node):
    def __init__(self):
        super().__init__(ROS_NODE_NAME)

        # 声明PuppyPose相关参数
        self.declare_parameter('PuppyPose_Stand_roll', Stand['roll'])
        self.declare_parameter('PuppyPose_Stand_pitch', Stand['pitch'])
        self.declare_parameter('PuppyPose_Stand_yaw', Stand['yaw'])
        self.declare_parameter('PuppyPose_Stand_height', Stand['height'])
        self.declare_parameter('PuppyPose_Stand_x_shift', Stand['x_shift'])
        self.declare_parameter('PuppyPose_Stand_stance_x', Stand['stance_x'])
        self.declare_parameter('PuppyPose_Stand_stance_y', Stand['stance_y'])

        self.declare_parameter('PuppyPose_LieDown_roll', LieDown['roll'])
        self.declare_parameter('PuppyPose_LieDown_pitch', LieDown['pitch'])
        self.declare_parameter('PuppyPose_LieDown_yaw', LieDown['yaw'])
        self.declare_parameter('PuppyPose_LieDown_height', LieDown['height'])
        self.declare_parameter('PuppyPose_LieDown_x_shift', LieDown['x_shift'])
        self.declare_parameter('PuppyPose_LieDown_stance_x', LieDown['stance_x'])
        self.declare_parameter('PuppyPose_LieDown_stance_y', LieDown['stance_y'])

        self.declare_parameter('PuppyPose_LookDown_roll', LookDown['roll'])
        self.declare_parameter('PuppyPose_LookDown_pitch', LookDown['pitch'])
        self.declare_parameter('PuppyPose_LookDown_yaw', LookDown['yaw'])
        self.declare_parameter('PuppyPose_LookDown_height', LookDown['height'])
        self.declare_parameter('PuppyPose_LookDown_x_shift', LookDown['x_shift'])
        self.declare_parameter('PuppyPose_LookDown_stance_x', LookDown['stance_x'])
        self.declare_parameter('PuppyPose_LookDown_stance_y', LookDown['stance_y'])

        self.declare_parameter('PuppyPose_LookDown_10deg_roll', LookDown_10deg['roll'])
        self.declare_parameter('PuppyPose_LookDown_10deg_pitch', LookDown_10deg['pitch'])
        self.declare_parameter('PuppyPose_LookDown_10deg_yaw', LookDown_10deg['yaw'])
        self.declare_parameter('PuppyPose_LookDown_10deg_height', LookDown_10deg['height'])
        self.declare_parameter('PuppyPose_LookDown_10deg_x_shift', LookDown_10deg['x_shift'])
        self.declare_parameter('PuppyPose_LookDown_10deg_stance_x', LookDown_10deg['stance_x'])
        self.declare_parameter('PuppyPose_LookDown_10deg_stance_y', LookDown_10deg['stance_y'])

        self.declare_parameter('PuppyPose_LookDown_20deg_roll', LookDown_20deg['roll'])
        self.declare_parameter('PuppyPose_LookDown_20deg_pitch', LookDown_20deg['pitch'])
        self.declare_parameter('PuppyPose_LookDown_20deg_yaw', LookDown_20deg['yaw'])
        self.declare_parameter('PuppyPose_LookDown_20deg_height', LookDown_20deg['height'])
        self.declare_parameter('PuppyPose_LookDown_20deg_x_shift', LookDown_20deg['x_shift'])
        self.declare_parameter('PuppyPose_LookDown_20deg_stance_x', LookDown_20deg['stance_x'])
        self.declare_parameter('PuppyPose_LookDown_20deg_stance_y', LookDown_20deg['stance_y'])

        self.declare_parameter('PuppyPose_LookDown_30deg_roll', LookDown_30deg['roll'])
        self.declare_parameter('PuppyPose_LookDown_30deg_pitch', LookDown_30deg['pitch'])
        self.declare_parameter('PuppyPose_LookDown_30deg_yaw', LookDown_30deg['yaw'])
        self.declare_parameter('PuppyPose_LookDown_30deg_height', LookDown_30deg['height'])
        self.declare_parameter('PuppyPose_LookDown_30deg_x_shift', LookDown_30deg['x_shift'])
        self.declare_parameter('PuppyPose_LookDown_30deg_stance_x', LookDown_30deg['stance_x'])
        self.declare_parameter('PuppyPose_LookDown_30deg_stance_y', LookDown_30deg['stance_y'])

        self.declare_parameter('PuppyPose_StandLow_roll', StandLow['roll'])
        self.declare_parameter('PuppyPose_StandLow_pitch', StandLow['pitch'])
        self.declare_parameter('PuppyPose_StandLow_yaw', StandLow['yaw'])
        self.declare_parameter('PuppyPose_StandLow_height', StandLow['height'])
        self.declare_parameter('PuppyPose_StandLow_x_shift', StandLow['x_shift'])
        self.declare_parameter('PuppyPose_StandLow_stance_x', StandLow['stance_x'])
        self.declare_parameter('PuppyPose_StandLow_stance_y', StandLow['stance_y'])

        # 声明GaitConfig相关参数
        self.declare_parameter('GaitConfigFast_overlap_time', GaitConfigFast['overlap_time'])
        self.declare_parameter('GaitConfigFast_swing_time', GaitConfigFast['swing_time'])
        self.declare_parameter('GaitConfigFast_clearance_time', GaitConfigFast['clearance_time'])
        self.declare_parameter('GaitConfigFast_z_clearance', GaitConfigFast['z_clearance'])

        self.declare_parameter('GaitConfigSlow_overlap_time', GaitConfigSlow['overlap_time'])
        self.declare_parameter('GaitConfigSlow_swing_time', GaitConfigSlow['swing_time'])
        self.declare_parameter('GaitConfigSlow_clearance_time', GaitConfigSlow['clearance_time'])
        self.declare_parameter('GaitConfigSlow_z_clearance', GaitConfigSlow['z_clearance'])

        self.declare_parameter('GaitConfigMarkTime_overlap_time', GaitConfigMarkTime['overlap_time'])
        self.declare_parameter('GaitConfigMarkTime_swing_time', GaitConfigMarkTime['swing_time'])
        self.declare_parameter('GaitConfigMarkTime_clearance_time', GaitConfigMarkTime['clearance_time'])
        self.declare_parameter('GaitConfigMarkTime_z_clearance', GaitConfigMarkTime['z_clearance'])

        self.declare_parameter('GaitConfig_current_overlap_time', GaitConfig['overlap_time'])
        self.declare_parameter('GaitConfig_current_swing_time', GaitConfig['swing_time'])
        self.declare_parameter('GaitConfig_current_clearance_time', GaitConfig['clearance_time'])
        self.declare_parameter('GaitConfig_current_z_clearance', GaitConfig['z_clearance'])

        # 声明其他参数
        self.declare_parameter('joint_state_pub_topic', 'false')
        self.declare_parameter('joint_state_controller_pub_topic', 'false')

        # 从参数中读取PuppyPose和GaitConfig
        self.PuppyPose = {
            'Stand': {
                'roll': self.get_parameter('PuppyPose_Stand_roll').get_parameter_value().double_value,
                'pitch': self.get_parameter('PuppyPose_Stand_pitch').get_parameter_value().double_value,
                'yaw': self.get_parameter('PuppyPose_Stand_yaw').get_parameter_value().double_value,
                'height': self.get_parameter('PuppyPose_Stand_height').get_parameter_value().double_value,
                'x_shift': self.get_parameter('PuppyPose_Stand_x_shift').get_parameter_value().double_value,
                'stance_x': self.get_parameter('PuppyPose_Stand_stance_x').get_parameter_value().integer_value,
                'stance_y': self.get_parameter('PuppyPose_Stand_stance_y').get_parameter_value().integer_value
            },
            'LieDown': {
                'roll': self.get_parameter('PuppyPose_LieDown_roll').get_parameter_value().double_value,
                'pitch': self.get_parameter('PuppyPose_LieDown_pitch').get_parameter_value().double_value,
                'yaw': self.get_parameter('PuppyPose_LieDown_yaw').get_parameter_value().double_value,
                'height': self.get_parameter('PuppyPose_LieDown_height').get_parameter_value().double_value,
                'x_shift': self.get_parameter('PuppyPose_LieDown_x_shift').get_parameter_value().double_value,
                'stance_x': self.get_parameter('PuppyPose_LieDown_stance_x').get_parameter_value().integer_value,
                'stance_y': self.get_parameter('PuppyPose_LieDown_stance_y').get_parameter_value().integer_value
            },
            'LookDown': {
                'roll': self.get_parameter('PuppyPose_LookDown_roll').get_parameter_value().double_value,
                'pitch': self.get_parameter('PuppyPose_LookDown_pitch').get_parameter_value().double_value,
                'yaw': self.get_parameter('PuppyPose_LookDown_yaw').get_parameter_value().double_value,
                'height': self.get_parameter('PuppyPose_LookDown_height').get_parameter_value().double_value,
                'x_shift': self.get_parameter('PuppyPose_LookDown_x_shift').get_parameter_value().double_value,
                'stance_x': self.get_parameter('PuppyPose_LookDown_stance_x').get_parameter_value().integer_value,
                'stance_y': self.get_parameter('PuppyPose_LookDown_stance_y').get_parameter_value().integer_value
            },
            'LookDown_10deg': {
                'roll': self.get_parameter('PuppyPose_LookDown_10deg_roll').get_parameter_value().double_value,
                'pitch': self.get_parameter('PuppyPose_LookDown_10deg_pitch').get_parameter_value().double_value,
                'yaw': self.get_parameter('PuppyPose_LookDown_10deg_yaw').get_parameter_value().double_value,
                'height': self.get_parameter('PuppyPose_LookDown_10deg_height').get_parameter_value().double_value,
                'x_shift': self.get_parameter('PuppyPose_LookDown_10deg_x_shift').get_parameter_value().double_value,
                'stance_x': self.get_parameter('PuppyPose_LookDown_10deg_stance_x').get_parameter_value().integer_value,
                'stance_y': self.get_parameter('PuppyPose_LookDown_10deg_stance_y').get_parameter_value().integer_value
            },
            'LookDown_20deg': {
                'roll': self.get_parameter('PuppyPose_LookDown_20deg_roll').get_parameter_value().double_value,
                'pitch': self.get_parameter('PuppyPose_LookDown_20deg_pitch').get_parameter_value().double_value,
                'yaw': self.get_parameter('PuppyPose_LookDown_20deg_yaw').get_parameter_value().double_value,
                'height': self.get_parameter('PuppyPose_LookDown_20deg_height').get_parameter_value().double_value,
                'x_shift': self.get_parameter('PuppyPose_LookDown_20deg_x_shift').get_parameter_value().double_value,
                'stance_x': self.get_parameter('PuppyPose_LookDown_20deg_stance_x').get_parameter_value().integer_value,
                'stance_y': self.get_parameter('PuppyPose_LookDown_20deg_stance_y').get_parameter_value().integer_value
            },
            'LookDown_30deg': {
                'roll': self.get_parameter('PuppyPose_LookDown_30deg_roll').get_parameter_value().double_value,
                'pitch': self.get_parameter('PuppyPose_LookDown_30deg_pitch').get_parameter_value().double_value,
                'yaw': self.get_parameter('PuppyPose_LookDown_30deg_yaw').get_parameter_value().double_value,
                'height': self.get_parameter('PuppyPose_LookDown_30deg_height').get_parameter_value().double_value,
                'x_shift': self.get_parameter('PuppyPose_LookDown_30deg_x_shift').get_parameter_value().double_value,
                'stance_x': self.get_parameter('PuppyPose_LookDown_30deg_stance_x').get_parameter_value().integer_value,
                'stance_y': self.get_parameter('PuppyPose_LookDown_30deg_stance_y').get_parameter_value().integer_value
            },
            'StandLow': {
                'roll': self.get_parameter('PuppyPose_StandLow_roll').get_parameter_value().double_value,
                'pitch': self.get_parameter('PuppyPose_StandLow_pitch').get_parameter_value().double_value,
                'yaw': self.get_parameter('PuppyPose_StandLow_yaw').get_parameter_value().double_value,
                'height': self.get_parameter('PuppyPose_StandLow_height').get_parameter_value().double_value,
                'x_shift': self.get_parameter('PuppyPose_StandLow_x_shift').get_parameter_value().double_value,
                'stance_x': self.get_parameter('PuppyPose_StandLow_stance_x').get_parameter_value().integer_value,
                'stance_y': self.get_parameter('PuppyPose_StandLow_stance_y').get_parameter_value().integer_value
            }
        }

        self.GaitConfig = {
            'Fast': {
                'overlap_time': self.get_parameter('GaitConfigFast_overlap_time').get_parameter_value().double_value,
                'swing_time': self.get_parameter('GaitConfigFast_swing_time').get_parameter_value().double_value,
                'clearance_time': self.get_parameter('GaitConfigFast_clearance_time').get_parameter_value().double_value,
                'z_clearance': self.get_parameter('GaitConfigFast_z_clearance').get_parameter_value().double_value
            },
            'Slow': {
                'overlap_time': self.get_parameter('GaitConfigSlow_overlap_time').get_parameter_value().double_value,
                'swing_time': self.get_parameter('GaitConfigSlow_swing_time').get_parameter_value().double_value,
                'clearance_time': self.get_parameter('GaitConfigSlow_clearance_time').get_parameter_value().double_value,
                'z_clearance': self.get_parameter('GaitConfigSlow_z_clearance').get_parameter_value().double_value
            },
            'MarkTime': {
                'overlap_time': self.get_parameter('GaitConfigMarkTime_overlap_time').get_parameter_value().double_value,
                'swing_time': self.get_parameter('GaitConfigMarkTime_swing_time').get_parameter_value().double_value,
                'clearance_time': self.get_parameter('GaitConfigMarkTime_clearance_time').get_parameter_value().double_value,
                'z_clearance': self.get_parameter('GaitConfigMarkTime_z_clearance').get_parameter_value().double_value
            },
            'current': {
                'overlap_time': self.get_parameter('GaitConfig_current_overlap_time').get_parameter_value().double_value,
                'swing_time': self.get_parameter('GaitConfig_current_swing_time').get_parameter_value().double_value,
                'clearance_time': self.get_parameter('GaitConfig_current_clearance_time').get_parameter_value().double_value,
                'z_clearance': self.get_parameter('GaitConfig_current_z_clearance').get_parameter_value().double_value
            }
        }

        # 初始化HiwonderPuppy
        self.puppy = HiwonderPuppy(setServoPulse=setServoPulse, servoParams=PWMServoParams(), dof='8')
        self.mpu = MPU6050(self)
        self.puppy.imu = None  # 初始不使用IMU

        # 配置初始姿态和步态（使用蹲姿StandLow）
        initial_pose = self.PuppyPose['LieDown']
        self.puppy.stance_config(
            self.stance(
                initial_pose['stance_x'],
                initial_pose['stance_y'],
                initial_pose['height'],
                initial_pose['x_shift']
            ),
            initial_pose['pitch'],
            initial_pose['roll']
        )
        self.puppy.gait_config(
            overlap_time=self.GaitConfig['current']['overlap_time'],
            swing_time=self.GaitConfig['current']['swing_time'],
            clearance_time=self.GaitConfig['current']['clearance_time'],
            z_clearance=self.GaitConfig['current']['z_clearance']
        )

        self.puppy.start()
        self.puppy.move_stop(servo_run_time=500)
        self.ak = ArmIK()
        self.ak.setPitchRangeMoving((8.51, 0, 3.3), 500)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        setServoPulse(9, 1500, 300)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.3))

        # 创建服务
        self.create_service(SetBool, f'/{ROS_NODE_NAME}/set_running', self.set_running)
        self.create_service(Empty, f'/{ROS_NODE_NAME}/go_home', self.go_home)
        self.create_service(SetBool, f'/{ROS_NODE_NAME}/set_self_balancing', self.set_self_balancing)
        self.create_service(SetRunActionName, f'/{ROS_NODE_NAME}/runActionGroup', self.runActionGroupFun)
        self.create_service(SetBool, f'/{ROS_NODE_NAME}/set_mark_time', self.set_mark_time)

        # 创建订阅者
        self.create_subscription(Gait, f'/{ROS_NODE_NAME}/gait', self.GaitFun, 10)
        self.create_subscription(Velocity, f'/{ROS_NODE_NAME}/velocity', self.VelocityFun, 10)
        self.create_subscription(Velocity, f'/{ROS_NODE_NAME}/velocity_move', self.VelocityMoveFun, 10)
        self.create_subscription(Velocity, f'/{ROS_NODE_NAME}/velocity/autogait', self.VelocityAutogaitFun, 10)
        self.create_subscription(Twist, '/cmd_vel', self.Cmd_velFun, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.Cmd_vel_nav_Fun, 10)
        self.create_subscription(Pose, f'/{ROS_NODE_NAME}/pose', self.PoseFun, 10)
        self.create_subscription(Polygon, f'/{ROS_NODE_NAME}/fourLegsRelativeCoordControl', self.fourLegsRelativeCoordControlFun, 10)
        self.create_subscription(Float32MultiArray, f'/{ROS_NODE_NAME}/gait/pc', self.GaitPCFun, 10)
        self.create_subscription(SetServo, f'/{ROS_NODE_NAME}/setServo', self.SetServo_Fun, 10)

        # 创建发布者
        self.legs_coord_pub = self.create_publisher(Polygon, f'/{ROS_NODE_NAME}/legs_coord', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_state = JointState()
        self.joint_state.name = ['rf_joint1', 'lf_joint1', 'rb_joint1', 'lb_joint1',
                                 'rf_joint2', 'lf_joint2', 'rb_joint2', 'lb_joint2']
        command_topics = ["/puppy/joint1_position_controller/command",
                          "/puppy/joint2_position_controller/command",
                          "/puppy/joint3_position_controller/command",
                          "/puppy/joint4_position_controller/command",
                          "/puppy/joint5_position_controller/command",
                          "/puppy/joint6_position_controller/command",
                          "/puppy/joint7_position_controller/command",
                          "/puppy/joint8_position_controller/command"]

        self.joint_controller_publishers = []
        for topic in command_topics:
            self.joint_controller_publishers.append(self.create_publisher(Float64, topic, 10))

        # 创建buzzer发布者
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.2))
        buzzer_msg = BuzzerState()
        buzzer_msg.freq = 1900
        buzzer_msg.on_time = 0.1
        buzzer_msg.off_time = 0.9
        buzzer_msg.repeat = 1
        self.buzzer_pub.publish(buzzer_msg)

        # 创建定时器
        self.timer = self.create_timer(0.01, self.pub)  # 100Hz
        self.times = 0

        # 读取joint_state_pub_topic和joint_state_controller_pub_topic参数
        self.joint_state_pub_topic = self.get_parameter('joint_state_pub_topic').get_parameter_value().string_value
        self.joint_state_controller_pub_topic = self.get_parameter('joint_state_controller_pub_topic').get_parameter_value().string_value

    def stance(self, x=0, y=0, z=-11, x_shift=2):
        return np.array([
            [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
            [y, y, y, y],
            [z, z, z, z],
        ])  # 此array的组合方式不要去改变

    def SetServo_Fun(self, msg):
        pulse = max(500, min(msg.pulse, 2500))
        time = max(0, min(msg.time, 30000))
        setServoPulse(msg.id, pulse, time)

    def GaitFun(self, msg):
        global GaitConfig
        self.get_logger().debug(str(msg))
        GaitConfig = {
            'overlap_time': msg.overlap_time,
            'swing_time': msg.swing_time,
            'clearance_time': msg.clearance_time,
            'z_clearance': msg.z_clearance
        }
        self.puppy.gait_config(
            overlap_time=GaitConfig['overlap_time'],
            swing_time=GaitConfig['swing_time'],
            clearance_time=GaitConfig['clearance_time'],
            z_clearance=GaitConfig['z_clearance']
        )

    def GaitPCFun(self, msg):
        global GaitConfig
        self.get_logger().debug(str(msg))
        if msg.data[0] == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )
        else:
            gait_type = msg.data[0]
            if gait_type == 1:  # Trot
                GaitConfig['overlap_time'] = msg.data[2] / 4
                GaitConfig['swing_time'] = msg.data[2] / 4
                GaitConfig['clearance_time'] = 0
            elif gait_type == 2:  # Amble
                GaitConfig['overlap_time'] = msg.data[2] / 5
                GaitConfig['swing_time'] = msg.data[2] / 5
                GaitConfig['clearance_time'] = msg.data[2] / 10
            elif gait_type == 3:  # Walk
                GaitConfig['overlap_time'] = msg.data[2] / 6
                GaitConfig['swing_time'] = msg.data[2] / 6
                GaitConfig['clearance_time'] = msg.data[2] / 6

            GaitConfig['z_clearance'] = msg.data[1]
            self.puppy.gait_config(
                overlap_time=GaitConfig['overlap_time'],
                swing_time=GaitConfig['swing_time'],
                clearance_time=GaitConfig['clearance_time'],
                z_clearance=GaitConfig['z_clearance']
            )
            self.VelocityFun(Velocity(x=msg.data[3], y=msg.data[4], yaw_rate=msg.data[5]))

    def Cmd_velFun(self, msg):
        global PuppyPose
        self.get_logger().debug(str(msg))

        if abs(msg.linear.x) > 0.5 or abs(msg.angular.z) > 0.5:
            PuppyPose = self.PuppyPose['Stand'].copy()
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )

            if abs(msg.linear.x) > abs(msg.angular.z):
                self.VelocityFun(Velocity(x=16 * np.sign(msg.linear.x), y=0.0, yaw_rate=0.0))
            else:
                self.VelocityFun(Velocity(x=0, y=0, yaw_rate=np.radians(25) * np.sign(msg.angular.z)))
        elif msg.linear.x == 0 and msg.angular.z == 0:
            self.VelocityFun(Velocity(x=0.0, y=0.0, yaw_rate=0.0))

    def Cmd_vel_nav_Fun(self, msg):
        global PuppyPose
        PuppyPose = self.PuppyPose['Stand'].copy()
        self.puppy.stance_config(
            self.stance(
                PuppyPose['stance_x'],
                PuppyPose['stance_y'],
                PuppyPose['height'],
                PuppyPose['x_shift']
            ),
            PuppyPose['pitch'],
            PuppyPose['roll']
        )
        self.VelocityFun(Velocity(x=msg.linear.x * 100, y=0.0, yaw_rate=msg.angular.z))

    def VelocityMoveFun(self, msg):
        self.get_logger().debug(str(msg))
        if msg.x == 0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )
        else:
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate=msg.yaw_rate)

    def VelocityFun(self, msg):
        self.get_logger().debug(str(msg))
        if msg.x == -999:
            self.puppy.move(x=0.0, y=0.0, yaw_rate=0.0)
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )
        elif msg.x == 0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )
        elif abs(msg.x) <= 35 and abs(msg.y) == 0 and abs(msg.yaw_rate) <= np.radians(51):
            if msg.x > 0:
                self.puppy.stance_config(
                    self.stance(
                        PuppyPose['stance_x'],
                        PuppyPose['stance_y'],
                        PuppyPose['height'],
                        PuppyPose['x_shift'] - 0.8
                    ),
                    PuppyPose['pitch'],
                    PuppyPose['roll']
                )
            else:
                self.puppy.stance_config(
                    self.stance(
                        PuppyPose['stance_x'],
                        PuppyPose['stance_y'],
                        PuppyPose['height'],
                        PuppyPose['x_shift'] + 0.8
                    ),
                    PuppyPose['pitch'],
                    PuppyPose['roll']
                )
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate=msg.yaw_rate)

    def VelocityAutogaitFun(self, msg):
        self.get_logger().debug(str(msg))
        if msg.x == 0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )
        elif abs(msg.x) <= 35 and abs(msg.y) == 0 and abs(msg.yaw_rate) <= np.radians(51):
            # 计算步态参数
            if abs(msg.x) <= 10:
                overlap_time_x = 0.45 - abs(msg.x) * 0.023
                swing_time_x = 0.38 - abs(msg.x) * 0.0154
                clearance_time_x = swing_time_x - 0.04
            elif abs(msg.x) <= 15:
                overlap_time_x = 0.45 - abs(msg.x) * 0.023
                swing_time_x = 0.38 - abs(msg.x) * 0.0154
                clearance_time_x = 0
            else:
                overlap_time_x = 0.1
                swing_time_x = 0.15
                clearance_time_x = 0

            if abs(msg.yaw_rate) <= np.radians(10):
                overlap_time_yaw_rate = 0.23 - abs(msg.yaw_rate) * 0.37
                swing_time_yaw_rate = 0.36 - abs(msg.yaw_rate) * 0.74
                clearance_time_yaw_rate = swing_time_yaw_rate - 0.04
            elif abs(msg.yaw_rate) <= np.radians(20):
                overlap_time_yaw_rate = 0.23 - abs(msg.yaw_rate) * 0.37
                swing_time_yaw_rate = 0.41 - abs(msg.yaw_rate) * 0.74
                clearance_time_yaw_rate = 0
            else:
                overlap_time_yaw_rate = 0.1
                swing_time_yaw_rate = 0.15
                clearance_time_yaw_rate = 0

            GaitConfig['overlap_time'] = min(overlap_time_x, overlap_time_yaw_rate)
            GaitConfig['swing_time'] = min(swing_time_x, swing_time_yaw_rate)
            GaitConfig['clearance_time'] = min(clearance_time_x, clearance_time_yaw_rate)

            self.puppy.gait_config(
                overlap_time=GaitConfig['overlap_time'],
                swing_time=GaitConfig['swing_time'],
                clearance_time=GaitConfig['clearance_time'],
                z_clearance=GaitConfig['z_clearance']
            )

            if msg.x > 0:
                self.puppy.stance_config(
                    self.stance(
                        PuppyPose['stance_x'],
                        PuppyPose['stance_y'],
                        PuppyPose['height'],
                        PuppyPose['x_shift'] - 0.8
                    ),
                    PuppyPose['pitch'],
                    PuppyPose['roll']
                )
            else:
                self.puppy.stance_config(
                    self.stance(
                        PuppyPose['stance_x'],
                        PuppyPose['stance_y'],
                        PuppyPose['height'],
                        PuppyPose['x_shift'] + 0.8
                    ),
                    PuppyPose['pitch'],
                    PuppyPose['roll']
                )

            self.puppy.move(x=msg.x, y=msg.y, yaw_rate=msg.yaw_rate)

    def PoseFun(self, msg):
        global PuppyPose
        self.get_logger().debug(str(msg))
        if (abs(msg.roll) <= math.radians(31) and abs(msg.pitch) <= math.radians(31)
                and abs(msg.yaw) == 0 and -15 <= msg.height <= -5
                and abs(msg.stance_x) <= 5 and abs(msg.stance_y) <= 5
                and abs(msg.x_shift) <= 10):

            if msg.run_time != 0:
                self.puppy.move_stop(servo_run_time=msg.run_time)
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.01))
                self.puppy.servo_force_run()

            PuppyPose = {
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'height': msg.height,
                'x_shift': msg.x_shift,
                'stance_x': msg.stance_x,
                'stance_y': msg.stance_y
            }
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )

    def fourLegsRelativeCoordControlFun(self, msg):
        self.get_logger().debug(str(msg))
        rotated_foot_locations = np.zeros((3, 4))
        for idx, p in enumerate(msg.points):
            rotated_foot_locations[:, idx] = [p.x, p.y, p.z]

        joint_angles = self.puppy.fourLegsRelativeCoordControl(rotated_foot_locations)
        self.puppy.sendServoAngle(joint_angles)

    def runActionGroupFun(self, request, response):
        self.get_logger().debug(str(request))
        runActionGroup(request.name, request.wait)
        response.success = True
        response.message = request.name
        return response

    def pub(self):
        coord = self.puppy.get_coord()

        if self.times >= 100:
            self.times = 0
            msg = Polygon()
            for i in range(4):
                point = Point32()
                point.x = float(coord[0, i])
                point.y = float(coord[1, i])
                point.z = float(coord[2, i])
                msg.points.append(point)
            self.legs_coord_pub.publish(msg)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.002))
            # 更新参数
            self.joint_state_pub_topic = self.get_parameter('joint_state_pub_topic').get_parameter_value().string_value
            self.joint_state_controller_pub_topic = self.get_parameter('joint_state_controller_pub_topic').get_parameter_value().string_value

        self.times += 1

        if self.joint_state_pub_topic.lower() == 'true' or self.joint_state_controller_pub_topic.lower() == 'true':
            joint_angles = self.puppy.fourLegsRelativeCoordControl(coord / 100)
            data = sum([list(joint_angles[1, :]), list(joint_angles[2, :])], [])
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            for i in range(len(data)):
                if i > 3:
                    data[i] = 0.0695044662 * data[i] ** 3 - 0.0249173454 * data[i] ** 2 - 0.786456081 * data[i] + 1.5443387652 - math.pi / 2
                # 正确初始化Float64消息
                float64_msg = Float64(data=data[i])
                if self.joint_state_controller_pub_topic.lower() == 'true':
                    self.joint_controller_publishers[i].publish(float64_msg)
            if self.joint_state_pub_topic.lower() == 'true':
                self.joint_state.position = data
                self.joint_state_pub.publish(self.joint_state)

    def set_running(self, request, response):
        self.get_logger().debug(str(request))
        if request.data:
            self.puppy.move_stop(servo_run_time=500)
            self.puppy.start()
        else:
            self.puppy.move_stop(servo_run_time=500)
            self.puppy.end()
        response.success = True
        response.message = 'set_running'
        return response

    def set_self_balancing(self, request, response):
        self.get_logger().debug(str(request))
        if request.data:
            global PuppyPose
            PuppyPose = self.PuppyPose['StandLow'].copy()
            self.puppy.stance_config(
                self.stance(
                    PuppyPose['stance_x'],
                    PuppyPose['stance_y'],
                    PuppyPose['height'],
                    PuppyPose['x_shift']
                ),
                PuppyPose['pitch'],
                PuppyPose['roll']
            )

            self.puppy.move_stop(servo_run_time=500)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.01))
            self.puppy.servo_force_run()
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
            self.puppy.move_stop(servo_run_time=0)
            self.puppy.imu = self.mpu
        else:
            self.puppy.imu = None
        response.success = True
        response.message = 'set_self_balancing'
        return response

    def set_mark_time(self, request, response):
        self.get_logger().debug(str(request))
        if request.data:
            self.go_home()
            GaitConfig['overlap_time'] = self.get_parameter('GaitConfigMarkTime_overlap_time').get_parameter_value().double_value
            GaitConfig['swing_time'] = self.get_parameter('GaitConfigMarkTime_swing_time').get_parameter_value().double_value
            GaitConfig['clearance_time'] = self.get_parameter('GaitConfigMarkTime_clearance_time').get_parameter_value().double_value
            GaitConfig['z_clearance'] = self.get_parameter('GaitConfigMarkTime_z_clearance').get_parameter_value().double_value
            self.puppy.gait_config(
                overlap_time=GaitConfig['overlap_time'],
                swing_time=GaitConfig['swing_time'],
                clearance_time=GaitConfig['clearance_time'],
                z_clearance=GaitConfig['z_clearance']
            )
            self.puppy.move(x=0, y=0, yaw_rate=0)
        response.success = True
        response.message = 'set_mark_time'
        return response

    def go_home(self, request=None, response=None):
        global PuppyPose
        self.get_logger().debug('go_home')
        PuppyPose = self.PuppyPose['StandLow'].copy()
        self.puppy.stance_config(
            self.stance(
                PuppyPose['stance_x'],
                PuppyPose['stance_y'],
                PuppyPose['height'],
                PuppyPose['x_shift']
            ),
            PuppyPose['pitch'],
            PuppyPose['roll']
        )

        self.puppy.move_stop(servo_run_time=500)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.01))
        self.puppy.servo_force_run()
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        self.puppy.move_stop(servo_run_time=0)
        return Empty.Response()

def main(args=None):
    rclpy.init(args=args)
    puppy = PUPPY()
    try:
        rclpy.spin(puppy)
    except KeyboardInterrupt:
        pass
    finally:
        puppy.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



