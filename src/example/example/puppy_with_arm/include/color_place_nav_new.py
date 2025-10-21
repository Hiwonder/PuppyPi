#!/usr/bin/env python3
# coding=utf8
import sys
import cv2
import math
import time
import rclpy
import yaml
from rclpy.node import Node
from enum import Enum
import threading
import numpy as np
from sensor_msgs.msg import Image
from puppy_control_msgs.msg import Velocity, Pose, Gait
from cv_bridge import CvBridge
from sdk import Misc
from action_msgs.msg import GoalStatusArray
from puppy_control_msgs.srv import SetRunActionName

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse
from sdk.ArmMoveIK import ArmIK


def cv2_image2ros(image, frame_id='', node=None):
    """
    将opencv的图片转换为ROS2 Image消息
    """
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    if node:
        ros_image.header.stamp = node.get_clock().now().to_msg()
    ros_image.header.frame_id = frame_id
    return ros_image


def getAreaMaxContour(contours):
    """
    找出面积最大的轮廓
    """
    contour_area_max = 0
    area_max_contour = None

    for c in contours:
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:
                area_max_contour = c

    return area_max_contour, contour_area_max


Bend = {'roll': math.radians(0), 'pitch': math.radians(-17.0), 'yaw': 0.0, 'height': -10.0, 'x_shift': 0.5, 'stance_x': 0, 'stance_y': 0}
Stand = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.0, 'height': -10.0, 'x_shift': -1.0, 'stance_x': 0, 'stance_y': 0}
GaitConfig = {'overlap_time': 0.15, 'swing_time': 0.15, 'clearance_time': 0.0, 'z_clearance': 3.0}

PuppyPose = Bend.copy()
PuppyMove = {'x': 0, 'y': 0, 'yaw_rate': 0}

target_color = 'red'
nav_status = False  
image = None        
block_center_point = (0, 0)

AK = ArmIK()

class PuppyStatus(Enum):
    START = 0
    NORMAL = 1
    FOUND_TARGET = 2
    PLACE = 3
    STOP = 4
    END = 5


puppyStatus = PuppyStatus.START

size = (320, 240)
img_centerx = 320

def load_color_ranges(yaml_file):
    with open(yaml_file, 'r') as file:
        config = yaml.safe_load(file)
    return config['color_range_list']

class HandControlWithArmNode(Node):
    def __init__(self):
        super().__init__('navigation_status_listener')
        self.color_range_list = load_color_ranges('/home/ubuntu/software/lab_tool/lab_config.yaml')

        # 发布者和订阅者
        self.puppy_gait_config_pub = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.puppy_velocity_pub = self.create_publisher(Velocity, '/puppy_control/velocity', 10)
        self.puppy_pose_pub = self.create_publisher(Pose, '/puppy_control/pose', 10)
        self.result_publisher = self.create_publisher(Image, '~/image_result', 10)

        self.image_subscriber = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.goal_status_subscriber = self.create_subscription(GoalStatusArray, '/move_base/status', self.goal_status_callback, 10)

        # 服务
        self.run_action_group_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup')

        # 变量初始化
        self.image = None
        self.nav_status = False

        # 定时器，用于替代 while rclpy.ok() 的循环，定期调用图像处理逻辑
        self.timer = self.create_timer(0.1, self.run)  # 每100ms调用一次run方法

        # 线程
        self.th1 = threading.Thread(target=self.move, daemon=True)

        # 初始化状态
        self.init_pose()

    def init_pose(self):
        PuppyPose['stance_x'] = float(PuppyPose.get('stance_x', 0))
        PuppyPose['stance_y'] = float(PuppyPose.get('stance_y', 0))
        PuppyPose['x_shift'] = float(PuppyPose.get('x_shift', 0))
        PuppyPose['height'] = float(PuppyPose.get('height', 0))
        PuppyPose['roll'] = float(PuppyPose.get('roll', 0))
        PuppyPose['pitch'] = float(PuppyPose.get('pitch', 0))
        PuppyPose['yaw'] = float(PuppyPose.get('yaw', 0))

        run_time = int(500)

        self.puppy_pose_pub.publish(Pose(
            stance_x=PuppyPose['stance_x'],
            stance_y=PuppyPose['stance_y'],
            x_shift=PuppyPose['x_shift'],
            height=PuppyPose['height'],
            roll=PuppyPose['roll'],
            pitch=PuppyPose['pitch'],
            yaw=PuppyPose['yaw'],
            run_time=run_time
        ))

    def image_callback(self, ros_image):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(ros_image, "rgb8")

    def goal_status_callback(self, msg):
        if len(msg.status_list) > 0:
            status = msg.status_list[0].status
            if status == 3 and not self.nav_status:
                self.get_logger().info("Target arrived")
                self.th1.start()
                self.nav_status = True

    def move(self):
        global block_center_point
        global PuppyPose
        global puppyStatus

        while rclpy.ok():
            if puppyStatus == PuppyStatus.START:
                PuppyPose = Bend.copy()
                self.puppy_pose_pub.publish(Pose(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                                                 height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500))
                self.puppy_gait_config_pub.publish(Gait(overlap_time=GaitConfig['overlap_time'], swing_time=GaitConfig['swing_time'],
                                                        clearance_time=GaitConfig['clearance_time'], z_clearance=GaitConfig['z_clearance']))
                time.sleep(1)
                puppyStatus = PuppyStatus.NORMAL
            elif puppyStatus == PuppyStatus.NORMAL:
                if block_center_point[1] > 270 and abs(block_center_point[0] - img_centerx) < 70:
                    self.call_go_home()
                    time.sleep(0.1)
                    self.run_action_group('place.d6a')
                    puppyStatus = PuppyStatus.STOP
                else:
                    self.adjust_position()

            elif puppyStatus == PuppyStatus.STOP:
                AK.setPitchRangeMoving((8.51, 0, 3.3), 500)
                time.sleep(0.5)
                PuppyPose = Stand.copy()
                self.puppy_pose_pub.publish(Pose(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift'],
                                                 height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time=500))
                time.sleep(0.5)

    def adjust_position(self):
        value = block_center_point[0] - img_centerx
        if block_center_point[1] <= 250:
            PuppyMove['x'] = 10
            PuppyMove['yaw_rate'] = math.radians(0)
        elif abs(value) > 80:
            PuppyMove['x'] = 5
            PuppyMove['yaw_rate'] = math.radians(-11 * np.sign(value))
        elif abs(value) > 50:
            PuppyMove['x'] = 5
            PuppyMove['yaw_rate'] = math.radians(-5 * np.sign(value))
        elif block_center_point[1] <= 270:
            PuppyMove['x'] = 8
            PuppyMove['yaw_rate'] = math.radians(0)

        self.puppy_velocity_pub.publish(Velocity(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate']))

    def call_go_home(self):
        """
        异步调用“go_home”服务，让机器人返回初始位置
        """
        go_home_client = self.create_client(SetRunActionName, '/puppy_control/go_home')
        while not go_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('go_home service not available, waiting again...')

        request = SetRunActionName.Request()
        request.name = 'go_home'
        request.times = 1

        future = go_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("Successfully called go_home service.")
        else:
            self.get_logger().error("Failed to call go_home service.")

    def run_action_group(self, action_name):
        """
        调用运行动作组服务
        """
        if not self.run_action_group_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('runActionGroup service not available.')
            return

        request = SetRunActionName.Request()
        request.name = action_name
        request.times = 1

        future = self.run_action_group_srv.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Action group '{action_name}' executed.")
        else:
            self.get_logger().error(f"Failed to execute action group '{action_name}'.")

    def run(self):
        """
        定时器回调，用于图像处理并发布结果
        """
        global image, block_center_point

        if self.image is not None:
            img_copy = self.image.copy()
            img_h, img_w = self.image.shape[:2]
            frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
            frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
            bgr_image = cv2.cvtColor(frame_gb, cv2.COLOR_RGB2BGR)
            frame_lab_all = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LAB)

            max_area = 0
            areaMaxContour_max = 0
            frame_mask = cv2.inRange(
                frame_lab_all,
                (self.color_range_list[target_color]['min'][0],
                 self.color_range_list[target_color]['min'][1],
                 self.color_range_list[target_color]['min'][2]),
                (self.color_range_list[target_color]['max'][0],
                 self.color_range_list[target_color]['max'][1],
                 self.color_range_list[target_color]['max'][2])
            )

            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

            areaMaxContour, area_max = getAreaMaxContour(contours)
            if areaMaxContour is not None:
                if area_max > max_area:
                    max_area = area_max
                    areaMaxContour_max = areaMaxContour

            if max_area > 200:
                ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)
                centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
                centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
                radius = int(Misc.map(radius, 0, size[0], 0, img_w))

                block_center_point = (centerX, centerY)

                cv2.circle(bgr_image, (centerX, centerY), radius, (0, 255, 255), 2)

            # 发布处理后的图像
            self.result_publisher.publish(cv2_image2ros(cv2.resize(bgr_image, (640, 480)), 'navigation_status_listener', self))


def main(args=None):
    rclpy.init(args=args)
    node = HandControlWithArmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

