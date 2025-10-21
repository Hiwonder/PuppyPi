#!/usr/bin/env python3
# encoding: utf-8
import cv2
import sys
import os
import enum
import math  
import time
import queue
import rclpy
import threading
import numpy as np
import faulthandler
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sdk.common import vector_2d_angle, distance
from sdk.ArmMoveIK import ArmIK
from collections import deque
sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse

faulthandler.enable()

def distance(point_1, point_2):
    """
    计算两个点间的距离(calculate the distance between two points)
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)

def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def cv2_image2ros(image, frame_id='', node=None):
    """
    将opencv的图片转换为ROS2 Image消息
    """
    bridge = CvBridge()
    ros_image = bridge.cv2_to_imgmsg(image, encoding="rgb8")
    ros_image.header.stamp = node.get_clock().now().to_msg()
    ros_image.header.frame_id = frame_id
    return ros_image

class HandControlWithArmNode(Node):
    def __init__(self, name):
        super().__init__(name)  # ROS2 中的节点初始化
        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(static_image_mode=False,
                                        max_num_hands=1,
                                        min_detection_confidence=0.7,
                                        min_tracking_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils
        self.ak = ArmIK()
        self.last_time = time.time()
        self.last_out = 0

        self.frames = 0

        self.window_size = 5  # 定义滑动窗口大小（滤波窗口大小）
        self.distance_measurements = deque(maxlen=self.window_size)  # 创建一个队列用于存储最近的距离测量数据

        self.filtered_distance = 0
        self.image = None

        # 定时器用于定期处理图像和发布数据
        self.timer = self.create_timer(0.05, self.timer_callback)  # 每50ms执行一次

        self.image_subscriber = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10  # 缓存队列大小
        )
        self.result_publisher = self.create_publisher(Image, '~/image_result', 10)  # 图像处理结果发布

        # 启动线程用于伺服电机控制
        self.th = threading.Thread(target=self.move, daemon=True)
        self.th.start()

    def image_callback(self, ros_image):
        """
        图像订阅回调，接收ROS图像消息并强制转换为RGB8格式的OpenCV图像
        """
        bridge = CvBridge()
        try:
            # 强制转换成 RGB8 格式
            cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            
            # 将图像保存在 self.image 变量中
            self.image = cv_image

        except Exception as e:
            self.get_logger().info(f"Error converting image: {e}")


    def init_arm(self):
        """
        初始化机械臂位置
        """
        setServoPulse(9, 1500, 300)
        setServoPulse(10, 800, 300)
        setServoPulse(11, 850, 300)
        time.sleep(0.3)

    def move(self):
        """
        伺服电机控制线程，根据手指之间的距离移动机械臂
        """
        while rclpy.ok():
            if self.filtered_distance > 0 and self.filtered_distance < 100:
                out = np.interp(self.filtered_distance, [15, 90], [1500, 900])
                setServoPulse(9, int(out), 0)
            else:
                time.sleep(0.01)

    def timer_callback(self):
        """
        定时器回调函数，用于图像处理和发布图像
        """
        if self.image is not None:
            image_flip = cv2.flip(self.image, 1)
            image_re = cv2.resize(image_flip, (320, 240), interpolation=cv2.INTER_NEAREST)
            self.image = None
            bgr_image = cv2.cvtColor(image_re, cv2.COLOR_RGB2BGR)
            try:
                results = self.hands.process(image_re)
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # 绘制手部关键点连线
                        self.mpDraw.draw_landmarks(bgr_image, hand_landmarks, self.mpHands.HAND_CONNECTIONS)
                        landmarks = get_hand_landmarks(image_re, hand_landmarks.landmark)

                        # 计算拇指和食指尖之间的距离
                        cv2.line(bgr_image, (int(landmarks[4][0]), int(landmarks[4][1])),
                                 (int(landmarks[8][0]), int(landmarks[8][1])), (0, 255, 255), 2)
                        cv2.circle(bgr_image, (int(landmarks[8][0]), int(landmarks[8][1])), 4, (0, 255, 255), -1)
                        cv2.circle(bgr_image, (int(landmarks[4][0]), int(landmarks[4][1])), 4, (0, 255, 255), -1)

                        # 计算手指尖的距离
                        distanceSum = distance(landmarks[8], landmarks[4])

                        distanceSum = max(15, min(distanceSum, 90))
                        self.distance_measurements.append(distanceSum)
                        self.filtered_distance = np.mean(self.distance_measurements)
                        self.filtered_distance = round(self.filtered_distance, 2)

                        cv2.putText(bgr_image, 'DST: ' + str(self.filtered_distance), (5, 220),
                                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

                self.frames += 1
                delta_time = time.time() - self.last_time
                cur_fps = np.around(self.frames / delta_time, 1)
                cv2.putText(bgr_image, 'FPS: ' + str(cur_fps), (5, 30),
                            cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 255), 2)

            except Exception as e:
                self.get_logger().info(f"Error: {e}")

            # 发布处理后的图像
            self.result_publisher.publish(cv2_image2ros(cv2.resize(bgr_image, (640, 480)), self.get_name(), self))

def main(args=None):
    rclpy.init(args=args)
    node = HandControlWithArmNode('hand_control')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
