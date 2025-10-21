#!/usr/bin/python3
# coding=utf8
import sys
import yaml  
import os  
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import rclpy
import time
from threading import Timer
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState
from sensor_msgs.msg import Image
from puppy_control_msgs.srv import SetRunActionName
from sdk import Misc

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse


class ColorDetectWithArm(Node):
    def __init__(self):
        super().__init__('color_detect_with_arm')

        # 初始化 CvBridge 实例
        self.bridge = CvBridge()

        # 定义 YAML 文件路径
        yaml_file_path = '/home/ubuntu/software/lab_tool/lab_config.yaml'
        
        # 尝试从YAML文件中加载 color_range_list
        if not os.path.exists(yaml_file_path):
            self.get_logger().error(f"YAML file not found at {yaml_file_path}")
        else:
            try:
                with open(yaml_file_path, 'r') as file:
                    self.color_range_list = yaml.safe_load(file)['color_range_list']
                self.get_logger().info(f"Loaded color ranges: {self.color_range_list}")
            except Exception as e:
                self.get_logger().error(f"Failed to load color_range_list from YAML: {e}")
                self.color_range_list = {}

        # 创建发布者和服务客户端
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.runActionGroup_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup')

        # 创建图像订阅
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        # 使用定时器定期调用 move 函数
        self.create_timer(1.0, self.move)

        # 初始变量
        self.detect_color = 'None'
        self.color_list = []
        self.action_finish = True
        self.target_color = 'green'
        self.draw_color = (255, 255, 0)  # Yellow in BGR

    def init_move(self):
        while not self.runActionGroup_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        req = SetRunActionName.Request()
        req.name = 'look_down.d6a'
        self.runActionGroup_srv.call_async(req)

    def move(self):
        if self.detect_color == self.target_color:
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.1
            msg.off_time = 0.9
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

            self.action_finish = False
            time.sleep(0.8)

            req = SetRunActionName.Request()
            req.name = 'grab.d6a'
            self.runActionGroup_srv.call_async(req)
            time.sleep(0.5)
            self.set_servo_pulse(9, 1200, 300)
            time.sleep(0.3)
            self.set_servo_pulse(9, 1500, 300)

            req.name = 'look_down.d6a'
            self.runActionGroup_srv.call_async(req)
            time.sleep(0.8)
            
            self.detect_color = 'None'
            self.draw_color = (255, 255, 0)

        self.action_finish = True

    def run(self, img):
        size = (320, 240)
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)

        max_area = 0
        color_area_max = None
        areaMaxContour_max = 0

        if self.action_finish:
            for i in self.color_range_list:
                if i in ['red', 'green', 'blue']:
                    frame_mask = cv2.inRange(frame_lab, np.array(self.color_range_list[i]['min']), np.array(self.color_range_list[i]['max']))
                    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                    dilated[0:120, :] = 0
                    dilated[:, 0:80] = 0
                    dilated[:, 240:320] = 0
                    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
                    areaMaxContour, area_max = self.get_area_max_contour(contours)

                    if areaMaxContour is not None and area_max > max_area:
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour

            if max_area > 4000:
                ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)
                centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
                centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
                radius = int(Misc.map(radius, 0, size[0], 0, img_w))
                cv2.circle(img, (centerX, centerY), radius, self.draw_color, 2)
                print(f"Detected Color: {color_area_max}, Center: ({centerX}, {centerY}), Radius: {radius}")
                
                self.detect_color = color_area_max
                self.draw_color = (0, 0, 255) if color_area_max == 'red' else (0, 255, 0)
            else:
                print("No valid color detected")

        cv2.rectangle(img, (190, 270), (450, 480), (0, 255, 255), 2)
        if self.detect_color == self.target_color:
            cv2.putText(img, "Target Color", (225, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, self.draw_color, 2)
        else:
            cv2.putText(img, "Not Target Color", (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, self.draw_color, 2)

        cv2.putText(img, "Color: " + self.detect_color, (225, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, self.draw_color, 2)
        return img

    def image_callback(self, ros_image):
        try:
            cv2_img = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            cv2_img = cv2.flip(cv2_img, 1)
            frame_result = self.run(cv2_img)
            cv2.imshow('Frame', frame_result)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def get_area_max_contour(self, contours):
        contour_area_max = 0
        area_max_contour = None
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 50:
                    area_max_contour = c
        return area_max_contour, contour_area_max

    def set_servo_pulse(self, id, pulse, time):
        setServoPulse(id, pulse, time)

def main(args=None):
    rclpy.init(args=args)
    color_detect_with_arm = ColorDetectWithArm()
    color_detect_with_arm.init_move()
    rclpy.spin(color_detect_with_arm)

    color_detect_with_arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
