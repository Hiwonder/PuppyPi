#!/usr/bin/python3
# coding=utf8
# 第7章 ROS机器狗创意课程\4.AI视觉巡线行走(7.ROS Robot Creative Lesson\4.AI Visual Line Follow Walking)
import cv2
import math
import time
import numpy as np
import yaml
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from puppy_control_msgs.msg import Velocity, Pose, Gait


def map_value(x, in_min, in_max, out_min, out_max):
    """映射函数，将 x 从 [in_min, in_max] 映射到 [out_min, out_max]"""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class VisualPatrolDemo(Node):
    def __init__(self):
        super().__init__('visual_patrol_demo')

        # 初始化变量
        self.is_shutdown = False
        self.__isRunning = False
        self.__target_color = 'red'  # 设置目标颜色为红色
        self.line_centerx = -1
        self.img_centerx = 320  # 图像中心点x坐标（320/2）

        # 定义 ROI 和权重
        self.roi = [
            (120, 140,  0, 320, 0.1), 
            (160, 180,  0, 320, 0.2), 
            (200, 220,  0, 320, 0.7)
        ]
        self.roi_h_list = [self.roi[0][0], self.roi[1][0] - self.roi[0][0], self.roi[2][0] - self.roi[1][0]]

        # 定义颜色范围 RGB 用于可视化
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }
        self.draw_color = self.range_rgb["black"]

        # 初始化 CvBridge
        self.bridge = CvBridge()

        # 读取颜色阈值配置
        self.color_range_list = self.load_color_ranges('/home/ubuntu/software/lab_tool/lab_config.yaml')

        # 获取 PuppyPose 参数
        self.PP = {
            'LookDown_20deg': {
                'roll': math.radians(0.0),
                'pitch': math.radians(-20.0),
                'yaw': 0.0,
                'height': -9.0,
                'x_shift': -0.1,
                'stance_x': 0.0,
                'stance_y': 0.0
            },
        }
        self.PuppyPose = self.PP['LookDown_20deg'].copy()

        # 定义步态配置
        self.GaitConfig = {
            'overlap_time': 0.1,
            'swing_time': 0.15,
            'clearance_time': 0.0,
            'z_clearance': 3.0
        }

        # 创建发布者
        self.PuppyGaitConfigPub = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.PuppyVelocityPub = self.create_publisher(Velocity, '/puppy_control/velocity', 10)
        self.PuppyPosePub = self.create_publisher(Pose, '/puppy_control/pose', 10)

        # 创建订阅者
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # 初始化 PuppyMove
        self.PuppyMove = {'x': 0.0, 'y': 0.0, 'yaw_rate': 0.0}
        self.get_logger().info(f"PuppyMove initialized: {self.PuppyMove}")

        # 发布初始姿态和步态配置
        self.initialize_robot()

        # 设置运行标志位
        self.__isRunning = True

        self.get_logger().info("视觉巡线节点初始化完成，开始运行。")

    def load_color_ranges(self, yaml_path):
        """加载颜色阈值配置"""
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                lab_config = yaml.safe_load(f)
                color_ranges = lab_config.get('color_range_list', {})
                if 'red' not in color_ranges:
                    self.get_logger().warn("颜色阈值配置中未找到 'red' 的阈值。")
                self.get_logger().info(f"颜色阈值加载成功: {list(color_ranges.keys())}")
                return color_ranges
        except Exception as e:
            self.get_logger().error(f"无法加载颜色阈值配置文件: {e}")
            return {}

    def initialize_robot(self):
        """发布初始姿态和步态配置，并调用 go_home 服务"""
        # 发布初始速度为零
        velocity_msg = Velocity()
        velocity_msg.x = 0.0
        velocity_msg.y = 0.0
        velocity_msg.yaw_rate = 0.0
        self.PuppyVelocityPub.publish(velocity_msg)
        self.get_logger().info("发布初始速度为零。")
        time.sleep(0.2)

        # 调用 go_home 服务回到初始位置
        go_home_client = self.create_client(Empty, '/puppy_control/go_home')
        while not go_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待服务 /puppy_control/go_home 可用...')
        request = Empty.Request()
        future = go_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("调用服务 /puppy_control/go_home 成功。")
        else:
            self.get_logger().error("调用服务 /puppy_control/go_home 失败。")

        # 发布初始姿态
        pose_msg = Pose()
        pose_msg.stance_x = float(self.PuppyPose['stance_x'])
        pose_msg.stance_y = float(self.PuppyPose['stance_y'])
        pose_msg.x_shift = float(self.PuppyPose['x_shift'])
        pose_msg.height = float(self.PuppyPose['height'])
        pose_msg.roll = float(self.PuppyPose['roll'])
        pose_msg.pitch = float(self.PuppyPose['pitch'])
        pose_msg.yaw = float(self.PuppyPose['yaw'])
        pose_msg.run_time = int(500)
        self.PuppyPosePub.publish(pose_msg)
        self.get_logger().info("发布初始姿态。")
        time.sleep(0.2)

        # 发布步态配置
        gait_msg = Gait()
        gait_msg.overlap_time = float(self.GaitConfig['overlap_time'])
        gait_msg.swing_time = float(self.GaitConfig['swing_time'])
        gait_msg.clearance_time = float(self.GaitConfig['clearance_time'])
        gait_msg.z_clearance = float(self.GaitConfig['z_clearance'])
        self.PuppyGaitConfigPub.publish(gait_msg)
        self.get_logger().info("发布步态配置。")
        time.sleep(0.2)

    def move_robot(self, error):
        """根据误差调整机器人的速度和转向"""
        if abs(error) <= 20:
            self.PuppyMove['x'] = 10.0
            self.PuppyMove['yaw_rate'] = 0.0
        else:
            self.PuppyMove['x'] = 8.0
            self.PuppyMove['yaw_rate'] = -0.004 * error  # 根据误差调整转向速度

        # 发布速度消息
        velocity_msg = Velocity()
        velocity_msg.x = float(self.PuppyMove['x'])
        velocity_msg.y = 0.0  # 保持 y 轴速度为 0
        velocity_msg.yaw_rate = float(self.PuppyMove['yaw_rate'])
        self.PuppyVelocityPub.publish(velocity_msg)
        self.get_logger().info(f"发布速度指令: x={velocity_msg.x}, yaw_rate={velocity_msg.yaw_rate}")

    def getAreaMaxContour(self, contours):
        """找出面积最大的轮廓"""
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 50:
                    area_max_contour = c

        return area_max_contour, contour_area_max

    def run(self, img):
        """处理图像，检测红色线条并计算中心位置"""
        size = (320, 240)
        img_h, img_w = img.shape[:2]

        frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_LINEAR)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)

        centroid_x_sum = 0
        weight_sum = 0

        # 绘制 ROI 区域用于调试
        # for r in self.roi:
        #     cv2.rectangle(img, (r[2], r[0]), (r[3], r[1]), (255, 0, 0), 1)

        # 将图像分割成上中下三个部分
        for idx, r in enumerate(self.roi):
            roi_h = self.roi_h_list[idx]
            blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
            frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 转换为 LAB 颜色空间

            detect_color = self.__target_color
            if detect_color in self.color_range_list:
                color_range = self.color_range_list[detect_color]
                frame_mask = cv2.inRange(frame_lab,
                                         np.array(color_range['min'], dtype=np.uint8),
                                         np.array(color_range['max'], dtype=np.uint8))
                # 形态学操作
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

                # 调试信息：显示掩模图像
                cv2.imshow('Mask', closed)
                cv2.waitKey(1)
            else:
                self.line_centerx = -1
                self.get_logger().warn(f"目标颜色 '{detect_color}' 不在颜色阈值列表中。")
                return img

            # 查找轮廓
            contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnt_large, area = self.getAreaMaxContour(contours)
            self.get_logger().debug(f"ROI {idx + 1}: 找到 {len(contours)} 个轮廓，最大面积为 {area}")

            if cnt_large is not None:
                rect = cv2.minAreaRect(cnt_large)
                box = np.int0(cv2.boxPoints(rect))
                for i in range(4):
                    box[i, 1] = box[i, 1] + r[0]
                    box[i, 1] = int(map_value(box[i, 1], 0, size[1], 0, img_h))
                for i in range(4):
                    box[i, 0] = int(map_value(box[i, 0], 0, size[0], 0, img_w))

                # 绘制轮廓和中心点
                cv2.drawContours(img, [box], -1, (0, 0, 255), 2)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2
                cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)

                # 计算加权中心
                centroid_x_sum += center_x * r[4]
                weight_sum += r[4]

                self.get_logger().debug(f"ROI {idx + 1}: center_x={center_x}, center_y={center_y}")
            else:
                self.get_logger().debug(f"ROI {idx + 1}: 未检测到有效轮廓。")
                continue  # 未检测到轮廓，跳过

        # 计算最终的中心点
        if weight_sum != 0:
            self.line_centerx = int(centroid_x_sum / weight_sum)
            cv2.circle(img, (self.line_centerx, int(center_y)), 10, (0, 255, 255), -1)
            self.get_logger().info(f'line_centerx: {self.line_centerx}')
        else:
            self.line_centerx = -1
            self.get_logger().warn("未检测到任何有效的线条。")

        return img

    def image_callback(self, ros_image):
        """图像回调函数，处理图像并发布运动指令"""
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv2_img = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        frame = cv2_img.copy()
        frame_result = frame

        # 处理图像并获取结果
        frame_result = self.run(frame)

        # 显示处理后的图像
        cv2.imshow('Frame', frame_result)
        cv2.waitKey(1)

        # 根据检测到的线条位置调整机器人运动
        if self.__isRunning and self.line_centerx != -1:
            error = self.line_centerx - self.img_centerx
            self.move_robot(error)
        elif self.__isRunning and self.line_centerx == -1:
            # 如果未检测到线条，停下机器人
            velocity_msg = Velocity()
            velocity_msg.x = 0.0
            velocity_msg.y = 0.0
            velocity_msg.yaw_rate = 0.0
            self.PuppyVelocityPub.publish(velocity_msg)
            self.get_logger().warn("未检测到线条，停下机器人。")

    def destroy_node(self):
        """节点销毁时发布停止指令"""
        self.is_shutdown = True
        velocity_msg = Velocity()
        velocity_msg.x = 0.0
        velocity_msg.y = 0.0
        velocity_msg.yaw_rate = 0.0
        self.PuppyVelocityPub.publish(velocity_msg)
        self.get_logger().info("节点销毁，发布停止指令。")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    visual_patrol_demo = VisualPatrolDemo()
    try:
        rclpy.spin(visual_patrol_demo)
    except KeyboardInterrupt:
        pass
    finally:
        visual_patrol_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
