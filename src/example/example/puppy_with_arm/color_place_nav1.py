#!/usr/bin/env python3
# coding=utf-8

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import cv2
import math
import threading
import numpy as np
import time
from enum import Enum

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from action_msgs.msg import GoalStatusArray

from puppy_control_msgs.msg import Velocity, Pose, Gait, SetServo
from puppy_control_msgs.srv import SetRunActionName

# 导入自定义模块
import sys
sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse
from sdk.ArmMoveIK import ArmIK
from sdk import Misc
from sdk import PID

from cv_bridge import CvBridge, CvBridgeError


def get_area_max_contour(contours):
    """
    找出面积最大的轮廓
    参数为要比较的轮廓的列表
    """
    contour_area_max = 0.0
    area_max_contour = None

    for c in contours:
        contour_area = math.fabs(cv2.contourArea(c))
        if contour_area > contour_area_max:
            if contour_area >= 5.0:  # 只有在面积大于5时，最大面积的轮廓才是有效的，以过滤干扰
                contour_area_max = contour_area
                area_max_contour = c

    return area_max_contour, contour_area_max


class PuppyStatus(Enum):
    START = 0
    NORMAL = 1  # 正常前进中
    FOUND_TARGET = 2  # 已经发现目标物块
    PLACE = 3  # 已经放置目标物块
    STOP = 4
    END = 5


class NavigationStatusListener(Node):
    def __init__(self):
        super().__init__('navigation_status_listener')

        # 初始化CvBridge
        self.bridge = CvBridge()

        # 加载颜色阈值
        config_path = '/home/ubuntu/ros2_ws/src/example/example/config/lab_config_list.yaml'
        try:
            with open(config_path, 'r') as file:
                import yaml
                config = yaml.safe_load(file)
                self.color_range_list = config.get('color_range_list', {})
                if not self.color_range_list:
                    self.get_logger().error("未找到 'color_range_list' 配置")
                    rclpy.shutdown()
        except FileNotFoundError:
            self.get_logger().error(f"配置文件不存在: {config_path}")
            rclpy.shutdown()

        # 初始化变量
        self.Bend = {
            'roll': math.radians(0.0),
            'pitch': math.radians(-17.0),
            'yaw': 0.000,
            'height': -10.0,
            'x_shift': 0.5,
            'stance_x': 0.0,
            'stance_y': 0.0
        }
        self.Stand = {
            'roll': math.radians(0.0),
            'pitch': math.radians(0.0),
            'yaw': 0.000,
            'height': -10.0,
            'x_shift': -1.0,
            'stance_x': 0.0,
            'stance_y': 0.0
        }
        self.GaitConfig = {
            'overlap_time': 0.15,
            'swing_time': 0.15,
            'clearance_time': 0.0,
            'z_clearance': 3.0
        }
        self.PuppyPose = self.Bend.copy()
        self.PuppyMove = {'x': 0.0, 'y': 0.0, 'yaw_rate': 0.0}

        self.target_color = 'red'
        self.nav_status = False
        self.is_running = False
        self.image = None
        self.block_center_point = (0.0, 0.0)  # 物块中心点坐标

        self.AK = ArmIK()

        self.puppy_status = PuppyStatus.START

        self.size = (320, 240)
        self.img_centerx = 320  # 理论值是640/2 = 320具体值需要根据不同机器的实际运行情况微调一下

        # 初始化发布者
        qos_profile = QoSProfile(depth=10)
        self.puppy_gait_pub = self.create_publisher(Gait, '/puppy_control/gait', qos_profile)
        self.puppy_velocity_pub = self.create_publisher(Velocity, '/puppy_control/velocity', qos_profile)
        self.puppy_pose_pub = self.create_publisher(Pose, '/puppy_control/pose', qos_profile)
        self.result_publisher = self.create_publisher(Image, 'image_result', qos_profile)

        # 初始化服务客户端
        self.run_action_group_client = self.create_client(SetRunActionName, '/puppy_control/runActionGroup')
        while not self.run_action_group_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /puppy_control/runActionGroup 服务...')

        self.go_home_client = self.create_client(Empty, '/puppy_control/go_home')
        while not self.go_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /puppy_control/go_home 服务...')

        # 初始化订阅者
        self.goal_status_subscriber = self.create_subscription(GoalStatusArray, '/move_base/status', self.goal_status_callback, 10)
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            qos_profile
        )

        # 初始化定时器用于图像处理
        timer_period = 0.2  # 5Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 初始化姿态
        self.PuppyPose = self.Stand.copy()
        self.publish_pose()

        # 初始化机械臂
        self.AK.setPitchRangeMoving((8.51, 0.0, 3.3), 500)
        self.get_logger().info("机械臂初始化完成")

        # 启动移动逻辑线程
        self.move_thread = threading.Thread(target=self.move, daemon=True)
        self.move_thread.start()

    def cv2_image2ros(self, image, frame_id=''):
        """
        将OpenCV图像转换为ROS2图像消息
        """
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = frame_id
            return ros_image
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge转换错误: {e}")
            return None

    def publish_pose(self):
        """
        发布Puppy的姿态
        """
        pose_msg = Pose()
        pose_msg.stance_x = float(self.PuppyPose['stance_x'])
        pose_msg.stance_y = float(self.PuppyPose['stance_y'])
        pose_msg.x_shift = float(self.PuppyPose['x_shift'])
        pose_msg.height = float(self.PuppyPose['height'])
        pose_msg.roll = float(self.PuppyPose['roll'])
        pose_msg.pitch = float(self.PuppyPose['pitch'])
        pose_msg.yaw = float(self.PuppyPose['yaw'])
        pose_msg.run_time = 500  # 确保为int
        self.puppy_pose_pub.publish(pose_msg)
        self.get_logger().info(f"发布姿态: {pose_msg}")

    def publish_gait_config(self):
        """
        发布步态配置
        """
        gait_msg = Gait()
        gait_msg.overlap_time = float(self.GaitConfig['overlap_time'])
        gait_msg.swing_time = float(self.GaitConfig['swing_time'])
        gait_msg.clearance_time = float(self.GaitConfig['clearance_time'])
        gait_msg.z_clearance = float(self.GaitConfig['z_clearance'])
        self.puppy_gait_pub.publish(gait_msg)
        self.get_logger().info(f"发布步态配置: {gait_msg}")

    def image_callback(self, ros_image):
        """
        图像订阅回调函数
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='rgb8')
            self.image = cv_image
            self.get_logger().debug("接收到新图像")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            self.image = None  # 重置图像

    def goal_status_callback(self, msg):
        """
        目标状态订阅回调函数
        """
        if len(msg.status_list) > 0:
            status = msg.status_list[0].status
            if status == 3:  # 目标已经到达
                if not self.nav_status:
                    self.get_logger().info("发现目标，准备启动移动逻辑")
                    self.nav_status = True
                    self.puppy_status = PuppyStatus.START  # 触发移动逻辑

    def call_go_home(self):
        """
        调用go_home服务
        """
        request = Empty.Request()
        future = self.go_home_client.call_async(request)
        future.add_done_callback(self.go_home_response_callback)

    def go_home_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("调用go_home服务成功")
        except Exception as e:
            self.get_logger().error(f"调用go_home服务失败: {e}")

    def run_action_group(self, action_name, wait):
        """
        调用runActionGroup服务执行动作组
        """
        request = SetRunActionName.Request()
        request.name = action_name
        request.wait = wait
        future = self.run_action_group_client.call_async(request)
        future.add_done_callback(self.run_action_group_response_callback)

    def run_action_group_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("调用runActionGroup服务成功")
        except Exception as e:
            self.get_logger().error(f"调用runActionGroup服务失败: {e}")

    def move(self):
        """
        控制Puppy的移动逻辑
        """
        while rclpy.ok():
            if self.puppy_status == PuppyStatus.START:
                self.PuppyPose = self.Bend.copy()
                self.publish_pose()
                self.publish_gait_config()
                self.get_logger().info('Puppy状态: START -> NORMAL')
                self.puppy_status = PuppyStatus.NORMAL
                time.sleep(1)

            elif self.puppy_status == PuppyStatus.NORMAL:
                block_center = self.block_center_point
                self.get_logger().info(f"当前物块中心点: {block_center}")

                if block_center[1] > 270 and abs(block_center[0] - self.img_centerx) < 70:
                    # 已经发现目标物块且物块在线上
                    self.get_logger().info("发现目标，准备放置")
                    self.call_go_home()
                    self.run_action_group('place.d6a', True)
                    self.puppy_status = PuppyStatus.STOP
                else:
                    value = block_center[0] - self.img_centerx
                    if block_center[1] <= 250:
                        self.PuppyMove['x'] = 10.0
                        self.PuppyMove['yaw_rate'] = math.radians(0.0)
                    elif abs(value) > 80:
                        self.PuppyMove['x'] = 5.0
                        self.PuppyMove['yaw_rate'] = math.radians(-11.0 * np.sign(value))
                    elif abs(value) > 50:
                        self.PuppyMove['x'] = 5.0
                        self.PuppyMove['yaw_rate'] = math.radians(-5.0 * np.sign(value))
                    elif block_center[1] <= 270:
                        self.PuppyMove['x'] = 8.0
                        self.PuppyMove['yaw_rate'] = math.radians(0.0)
                    self.puppy_velocity_pub.publish(Velocity(
                        x=self.PuppyMove['x'],
                        y=self.PuppyMove['y'],
                        yaw_rate=self.PuppyMove['yaw_rate']
                    ))
                    self.get_logger().info(f"发布速度: {self.PuppyMove}")

            elif self.puppy_status == PuppyStatus.STOP:
                self.AK.setPitchRangeMoving((8.51, 0.0, 3.3), 500)
                time.sleep(0.5)
                self.PuppyPose = self.Stand.copy()
                self.publish_pose()
                time.sleep(0.5)
                self.puppy_status = PuppyStatus.END  # 假设END为停止状态

            time.sleep(0.1)  # 控制循环频率

    def timer_callback(self):
        """
        定时器回调函数，用于处理图像和发布处理后的图像
        """
        if self.image is not None:
            self.get_logger().debug("开始处理图像")
            img_copy = self.image.copy()
            img_h, img_w = self.image.shape[:2]
            frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
            frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
            bgr_image = cv2.cvtColor(frame_gb, cv2.COLOR_RGB2BGR)
            frame_lab_all = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LAB)  # 转换到LAB空间

            # 使用加载的颜色范围
            color_range = self.color_range_list.get(self.target_color, None)
            if color_range:
                lower_color = np.array(color_range['min'], dtype=np.uint8)
                upper_color = np.array(color_range['max'], dtype=np.uint8)
                frame_mask = cv2.inRange(frame_lab_all, lower_color, upper_color)
                self.get_logger().debug(f"应用颜色范围: min={lower_color}, max={upper_color}")
            else:
                self.get_logger().error(f"未定义颜色范围: {self.target_color}")
                frame_mask = np.zeros((self.size[1], self.size[0]), dtype=np.uint8)

            # 腐蚀和膨胀
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

            # 找出轮廓
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
            area_max_contour, area_max = get_area_max_contour(contours)

            self.get_logger().debug(f"找到 {len(contours)} 个轮廓，最大面积: {area_max}")

            if area_max_contour is not None and area_max > 200.0:
                ((centerX, centerY), radius) = cv2.minEnclosingCircle(area_max_contour)
                centerX = int(Misc.map(centerX, 0, self.size[0], 0, img_w))
                centerY = int(Misc.map(centerY, 0, self.size[1], 0, img_h))
                radius = int(Misc.map(radius, 0, self.size[0], 0, img_w))
                self.block_center_point = (float(centerX), float(centerY))
                cv2.circle(bgr_image, (centerX, centerY), radius, (0, 255, 255), 2)  # 画圆
                self.get_logger().info(f"检测到物块中心点: ({centerX}, {centerY}), 半径: {radius}")
            else:
                self.get_logger().debug("未检测到有效物块")

            # 发布处理后的图像
            ros_result_image = self.cv2_image2ros(cv2.resize(bgr_image, (640, 480)), 'navigation_status_listener')
            if ros_result_image is not None:
                self.result_publisher.publish(ros_result_image)
                self.get_logger().debug("发布处理后的图像")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationStatusListener()
    node.get_logger().info("NavigationStatusListener节点已启动")

    try:
        rclpy.spin(node, executor=rclpy.executors.MultiThreadedExecutor(num_threads=4))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("NavigationStatusListener节点已关闭")


if __name__ == '__main__':
    main()
