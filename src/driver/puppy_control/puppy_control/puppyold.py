#!/usr/bin/python3
# coding=utf8
# Author: Summer
# Email: 997950600@qq.com

import time
import os, sys, math
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

sys.path.append('/home/ubuntu/software/puppypi_control')
from servo_controller import setServoPulse, updatePulse
from action_group_control import runActionGroup, stopActionGroup
from puppy_kinematics import HiwonderPuppy, PWMServoParams

class PoseConfig:
    def __init__(self, roll=0.0, pitch=0.0, yaw=0.0, height=-10.0, x_shift=0.5, stance_x=0, stance_y=0):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.height = height
        self.x_shift = x_shift
        self.stance_x = stance_x
        self.stance_y = stance_y

class GaitConfig:
    def __init__(self, overlap_time=0.1, swing_time=0.15, clearance_time=0.0, z_clearance=5.0):
        self.overlap_time = overlap_time
        self.swing_time = swing_time
        self.clearance_time = clearance_time
        self.z_clearance = z_clearance

class MPU6050:
    def __init__(self, node):
        self.node = node
        self.subscriber = self.node.create_subscription(Imu, '/ros_robot_controller/imu_raw', self.GetImuFun, 10)
        self.data = {'accel': [0.0, 0.0, 0.0], 'gyro': [0.0, 0.0, 0.0]}
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
        x1, x2, y1, angle, K2 = 0, 0, 0, 0, 0.02

        def fun(angle_m, gyro_m, dt=0.01):
            nonlocal x1, x2, y1, angle, K2
            x1 = (angle_m - angle) * (1 - K2) * (1 - K2)
            y1 = y1 + x1 * dt
            x2 = y1 + 2 * (1 - K2) * (angle_m - angle) + gyro_m
            angle = angle + x2 * dt
            return angle

        return fun

    def get_euler_angle(self, dt=0.01):
        accel_Y = math.atan2(self.data['accel'][0], self.data['accel'][2]) * 180 / math.pi
        gyro_Y = self.data['gyro'][1]
        angleY = self.SecondOrderFilterY(-accel_Y, gyro_Y, dt)

        accel_X = math.atan2(self.data['accel'][1], self.data['accel'][2]) * 180 / math.pi
        gyro_X = self.data['gyro'][0]
        angleX = self.SecondOrderFilterX(accel_X, gyro_X, dt)

        return PoseConfig(roll=-math.radians(angleY), pitch=-math.radians(angleX), yaw=0)

class PUPPY(Node):
    def __init__(self):
        super().__init__('puppy_control')

        # 初始化姿态和步态配置
        self.PuppyPose = PoseConfig(height=-5, x_shift=2)
        self.GaitConfig = GaitConfig()
        self.times = 0

        # 声明和初始化参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('joint_state_pub_topic', 'false'),
                ('joint_state_controller_pub_topic', 'false')
            ])

        self.joint_state_pub_topic = self.get_parameter('joint_state_pub_topic').get_parameter_value().string_value
        self.joint_state_controller_pub_topic = self.get_parameter('joint_state_controller_pub_topic').get_parameter_value().string_value

        # 初始化 PUPPY 控制器
        self.puppy = HiwonderPuppy(setServoPulse=setServoPulse, servoParams=PWMServoParams(), dof='8')
        self.mpu = MPU6050(self)
        self.puppy.imu = None

        # 初始化姿态和步态
        self.configure_stance()
        self.configure_gait()

        self.puppy.start()
        self.puppy.move_stop(servo_run_time=500)

        # ROS2 服务和话题订阅
        self.create_service(SetBool, '/puppy_control/set_running', self.set_running)
        self.create_service(Empty, '/puppy_control/go_home', self.go_home)
        self.create_service(SetBool, '/puppy_control/set_self_balancing', self.set_self_balancing)
        self.create_service(SetRunActionName, '/puppy_control/runActionGroup', self.runActionGroupFun)
        self.create_service(SetBool, '/puppy_control/set_mark_time', self.set_mark_time)

        self.velocity_pub = self.create_publisher(Polygon, '/puppy_control/legs_coord', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        command_topics = [
            "/puppy/joint1_position_controller/command",
            "/puppy/joint2_position_controller/command",
            "/puppy/joint3_position_controller/command",
            "/puppy/joint4_position_controller/command",
            "/puppy/joint5_position_controller/command",
            "/puppy/joint6_position_controller/command",
            "/puppy/joint7_position_controller/command",
            "/puppy/joint8_position_controller/command"
        ]

        self.joint_controller_publishers = [
            self.create_publisher(Float64, topic, 10) for topic in command_topics
        ]

        self.create_subscription(Gait, '/puppy_control/gait', self.GaitFun, 10)
        self.create_subscription(Velocity, '/puppy_control/velocity', self.VelocityFun, 10)
        self.create_subscription(Velocity, '/puppy_control/velocity_move', self.VelocityMoveFun, 10)
        self.create_subscription(Velocity, '/puppy_control/velocity/autogait', self.VelocityAutogaitFun, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.Cmd_vel_nav_Fun, 9)
        self.create_subscription(Pose, '/puppy_control/pose', self.PoseFun, 10)
        self.create_subscription(Polygon, '/puppy_control/fourLegsRelativeCoordControl', self.fourLegsRelativeCoordControlFun, 10)
        self.create_subscription(Float32MultiArray, '/puppy_control/gait/pc', self.GaitPCFun, 10)
        self.create_subscription(SetServo, '/puppy_control/setServo', self.SetServo_Fun, 10)

        self.get_logger().info("PUPPY node initialized")

        # 初始化蜂鸣器
        buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.get_clock().sleep_for(Duration(seconds=0.2))
        msg = BuzzerState(freq=1900, on_time=0.1, off_time=0.9, repeat=1)
        buzzer_pub.publish(msg)

        # 启动定时器
        self.create_timer(0.01, self.pub)

    def configure_stance(self):
        self.puppy.stance_config(
            self.stance(self.PuppyPose.stance_x, self.PuppyPose.stance_y, self.PuppyPose.height, self.PuppyPose.x_shift), 
            self.PuppyPose.pitch, 
            self.PuppyPose.roll
        )

    def configure_gait(self):
        self.puppy.gait_config(
            overlap_time=self.GaitConfig.overlap_time, 
            swing_time=self.GaitConfig.swing_time, 
            clearance_time=self.GaitConfig.clearance_time, 
            z_clearance=self.GaitConfig.z_clearance
        )

    def stance(self, x=0, y=0, z=-11, x_shift=2):
        return np.array([
            [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
            [y, y, y, y],
            [z, z, z, z],
        ])

    def SetServo_Fun(self, msg):
        pulse = min(max(msg.pulse, 500), 2500)
        time = min(msg.time, 30000)
        setServoPulse(msg.id, pulse, time)

    def GaitFun(self, msg):
        self.get_logger().info(f"Received Gait: {msg}")
        self.GaitConfig.overlap_time = msg.overlap_time
        self.GaitConfig.swing_time = msg.swing_time
        self.GaitConfig.clearance_time = msg.clearance_time
        self.GaitConfig.z_clearance = msg.z_clearance
        self.configure_gait()

    def GaitPCFun(self, msg):
        self.get_logger().info(f"Received GaitPC: {msg}")
        if msg.data[0] == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.configure_stance()
        else:
            period = msg.data[2]
            if msg.data[0] == 1:  # Trot
                self.GaitConfig.overlap_time = period / 4
                self.GaitConfig.swing_time = period / 4
                self.GaitConfig.clearance_time = 0
            elif msg.data[0] == 2:  # Amble
                self.GaitConfig.overlap_time = period / 5
                self.GaitConfig.swing_time = period / 5
                self.GaitConfig.clearance_time = period / 10
            elif msg.data[0] == 3:  # Walk
                self.GaitConfig.overlap_time = period / 6
                self.GaitConfig.swing_time = period / 6
                self.GaitConfig.clearance_time = period / 6

            self.GaitConfig.z_clearance = msg.data[1]
            self.configure_gait()
            self.VelocityFun(Velocity(x=msg.data[3], y=msg.data[4], yaw_rate=msg.data[5]))

    def Cmd_velFun(self, msg):
        self.get_logger().info(f"Received cmd_vel: {msg}")
        self.PuppyPose = PoseConfig()  # Reset to default Pose
        self.configure_stance()

        if abs(msg.linear.x) > abs(msg.angular.z):
            self.VelocityFun(Velocity(16 * np.sign(msg.linear.x), 0, 0))
        else:
            self.VelocityFun(Velocity(0, 0, np.radians(25) * np.sign(msg.angular.z)))

    def Cmd_vel_nav_Fun(self, msg):
        self.get_logger().info(f"Received cmd_vel_nav: {msg}")
        self.PuppyPose = PoseConfig()
        self.configure_stance()
        self.VelocityFun(Velocity(x=msg.linear.x * 100, y=0.0, yaw_rate=msg.angular.z))

    def VelocityMoveFun(self, msg):
        self.get_logger().info(f"Received VelocityMove: {msg}")
        if msg.x == 0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.configure_stance()
        else:
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate=msg.yaw_rate)

    def VelocityFun(self, msg):
        self.get_logger().info(f"Received Velocity: {msg}")
        if msg.x == -999:  # 原地踏步
            self.puppy.move(x=0, y=0, yaw_rate=0)
            self.configure_stance()
        elif msg.x == 0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.configure_stance()
        else:
            x_shift_adjust = -0.8 if msg.x > 0 else 0.8
            self.PuppyPose.x_shift += x_shift_adjust
            self.configure_stance()
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate=msg.yaw_rate)

    def VelocityAutogaitFun(self, msg):
        self.get_logger().info(f"Received VelocityAutogait: {msg}")
        if msg.x == 0 and msg.y == 0 and msg.yaw_rate == 0:
            self.puppy.move_stop(servo_run_time=100)
            self.configure_stance()
        else:
            overlap_time_x, swing_time_x, clearance_time_x = self.calculate_gait_parameters(msg.x)
            overlap_time_yaw_rate, swing_time_yaw_rate, clearance_time_yaw_rate = self.calculate_gait_parameters(msg.yaw_rate, is_yaw=True)

            self.GaitConfig.overlap_time = min(overlap_time_x, overlap_time_yaw_rate)
            self.GaitConfig.swing_time = min(swing_time_x, swing_time_yaw_rate)
            self.GaitConfig.clearance_time = min(clearance_time_x, clearance_time_yaw_rate)

            self.configure_gait()

            x_shift_adjust = -0.8 if msg.x > 0 else 0.8
            self.PuppyPose.x_shift += x_shift_adjust
            self.configure_stance()
            self.puppy.move(x=msg.x, y=msg.y, yaw_rate=msg.yaw_rate)

    def calculate_gait_parameters(self, value, is_yaw=False):
        if is_yaw:
            if abs(value) <= np.radians(10):
                overlap_time = 0.23 - abs(value) * 0.37
                swing_time = 0.36 - abs(value) * 0.74
                clearance_time = swing_time - 0.04
            elif abs(value) <= np.radians(20):
                overlap_time = 0.23 - abs(value) * 0.37
                swing_time = 0.41 - abs(value) * 0.74
                clearance_time = 0
            else:
                overlap_time = 0.1
                swing_time = 0.15
                clearance_time = 0
        else:
            if abs(value) <= 10:
                overlap_time = 0.45 - abs(value) * 0.023
                swing_time = 0.38 - abs(value) * 0.0154
                clearance_time = swing_time - 0.04
            elif abs(value) <= 15:
                overlap_time = 0.45 - abs(value) * 0.023
                swing_time = 0.38 - abs(value) * 0.0154
                clearance_time = 0
            else:
                overlap_time = 0.1
                swing_time = 0.15
                clearance_time = 0

        return overlap_time, swing_time, clearance_time

    def PoseFun(self, msg):
        self.get_logger().info(f"Received Pose: {msg}")
        if (abs(msg.roll) <= math.radians(31) and abs(msg.pitch) <= math.radians(31)
                and abs(msg.yaw) == 0 and -15 <= msg.height <= -5
                and abs(msg.stance_x) <= 5 and abs(msg.stance_y) <= 5
                and abs(msg.x_shift) <= 10):

            if msg.run_time != 0:
                self.puppy.move_stop(servo_run_time=msg.run_time)
                self.get_clock().sleep_for(Duration(seconds=0.01))
                self.puppy.servo_force_run()

            self.PuppyPose = PoseConfig(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw, height=msg.height, 
                                        x_shift=msg.x_shift, stance_x=msg.stance_x, stance_y=msg.stance_y)
            self.configure_stance()

    def fourLegsRelativeCoordControlFun(self, msg):
        self.get_logger().info(f"Received fourLegsRelativeCoordControl: {msg}")
        rotated_foot_locations = np.zeros((3, 4))
        for idx, p in enumerate(msg.points):
            rotated_foot_locations[:, idx] = p.x, p.y, p.z

        joint_angles = self.puppy.fourLegsRelativeCoordControl(rotated_foot_locations)
        self.puppy.sendServoAngle(joint_angles)

    def runActionGroupFun(self, msg):
        self.get_logger().info(f"Received runActionGroup: {msg}")
        runActionGroup(msg.name, msg.wait)
        return [True, msg.name]

    def pub(self):
        self.times += 1
        coord = self.puppy.get_coord()

        if self.times >= 100:
            self.times = 0

            # 发布 Polygon 消息
            msg = Polygon(points=[Point32(x=coord[0, i], y=coord[1, i], z=coord[2, i]) for i in range(4)])
            self.velocity_pub.publish(msg)

            # 计算关节角度并准备发布 JointState 消息
            joint_angles = self.puppy.fourLegsRelativeCoordControl(coord / 100)
            data = sum([list(joint_angles[1, :]), list(joint_angles[2, :])], [])

            if self.joint_state_pub_topic or self.joint_state_controller_pub_topic:
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()

                for i in range(len(data)):
                    if i > 3:
                        data[i] = 0.0695044662 * data[i] ** 3 - 0.0249173454 * data[i] ** 2 - 0.786456081 * data[i] + 1.5443387652 - math.pi / 2

                    if self.joint_state_controller_pub_topic:
                        self.joint_controller_publishers[i].publish(Float64(data=data[i]))

                if self.joint_state_pub_topic:
                    joint_state_msg.position = data 
                    self.joint_state_pub.publish(joint_state_msg)

    def set_running(self, request, response):
        self.get_logger().debug(f"Received set_running request: {request}")
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
        self.get_logger().debug(f"Received set_self_balancing request: {request}")
        if request.data:
            self.PuppyPose = PoseConfig()
            self.configure_stance()
            self.puppy.move_stop(servo_run_time=500)
            self.get_clock().sleep_for(Duration(seconds=0.01))
            self.puppy.servo_force_run()
            self.get_clock().sleep_for(Duration(seconds=0.5))
            self.puppy.move_stop(servo_run_time=0)
            self.puppy.imu = self.mpu
        else:
            self.puppy.imu = None
        response.success = True
        response.message = 'set_self_balancing'
        return response

    def set_mark_time(self, request, response):
        self.get_logger().debug(f"Received set_mark_time request: {request}")
        if request.data:
            self.go_home()
            self.GaitConfig = GaitConfig(overlap_time=0.2, swing_time=0.1, clearance_time=0.0, z_clearance=5)
            self.configure_gait()
            self.puppy.move(x=0, y=0, yaw_rate=0)
        response.success = True
        response.message = 'set_mark_time'
        return response

    def go_home(self, request=None, response=None):
        self.get_logger().debug('Executing go_home')
        self.PuppyPose = PoseConfig()
        self.configure_stance()
        self.puppy.move_stop(servo_run_time=500)
        self.get_clock().sleep_for(Duration(seconds=0.01))
        self.puppy.servo_force_run()
        self.get_clock().sleep_for(Duration(seconds=0.5))
        self.puppy.move_stop(servo_run_time=0)
        if response is not None:
            response.success = True
            response.message = 'go_home'
            return response

def main(args=None):
    rclpy.init(args=args)
    try:
        puppy_node = PUPPY()
        rclpy.spin(puppy_node)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        puppy_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


