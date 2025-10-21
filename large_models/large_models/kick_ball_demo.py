#!/usr/bin/python3
# coding=utf8
import os
import sys
import cv2
import time
import math
import threading
import numpy as np
from enum import Enum
import yaml
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from puppy_control_msgs.msg import Velocity, Pose, Gait
from puppy_control_msgs.srv import SetRunActionName
from large_models_msgs.srv import SetString
from std_srvs.srv import SetBool, Trigger, Empty
from std_srvs.srv import Trigger, SetBool
from rclpy.executors import MultiThreadedExecutor

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min

def emptyFunc(img=None):
    return img

def setRange(x, x_min, x_max):
    tmp = x if x > x_min else x_min
    tmp = tmp if tmp < x_max else x_max
    return tmp

class PuppyStatus(Enum):
    LOOKING_FOR = 0  # 寻找
    LOOKING_FOR_LEFT = 1
    LOOKING_FOR_RIGHT = 2
    FOUND_TARGET = 3  # 已经发现目标
    CLOSE_TO_TARGET = 4  # 靠近目标
    CLOSE_TO_TARGET_FINE_TUNE = 5  # 细调
    KICK_BALL = 6  # 踢球
    STOP = 10
    END = 20

class KickBallDemo(Node):
    def __init__(self):
        super().__init__('kick_ball_demo')

        # Initialize variables
        self.is_shutdown = False
        self.debug = False
        self.haved_detect = False
        
        self.start_time = 0
        self.elapsed_time = 0
        
        self.__target_color = ('red',)
        
        self.enable_running = False
        self.thread_started = True
        self.thread = None

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize status
        self.puppyStatus = PuppyStatus.LOOKING_FOR
        self.puppyStatusLast = PuppyStatus.END

        # Expected centers
        self.expect_center = {'X':640/2,'Y':480/2}
        self.expect_center_kick_ball_left = {'X':150,'Y':480-150}
        self.expect_center_kick_ball_right = {'X':640-150,'Y':480-150}

        self.target_info = None

        # Range RGB
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        self.color_list = []
        self.detect_color = 'None'
        self.draw_color = self.range_rgb["black"]

        # Size
        self.size = (320, 240)

        # Lock
        self.lock = threading.Lock()

        # Read color ranges from lab_config.yaml using yaml_handle.py logic
        with open('/home/ubuntu/software/lab_tool/lab_config.yaml', 'r', encoding='utf-8') as f:
            lab_config = yaml.safe_load(f)
            if 'color_range_list' in lab_config:
                self.color_range_list = lab_config['color_range_list']
            else:
                self.get_logger().error("lab_config.yaml does not contain 'color_range_list'")
                self.color_range_list = {}
        self.PP = {
            'LookDown_10deg': {
                'roll': math.radians(0),
                'pitch': math.radians(-15.0),
                'yaw': 0.0,
                'height': -9.0,
                'x_shift': -0.1,
                'stance_x': 0.0,  
                'stance_y': 0.0   
            },
            'LookDown_20deg': {
                'roll': math.radians(0),
                'pitch': math.radians(-20.0),
                'yaw': 0.0,
                'height': -9.0,
                'x_shift': -0.1,
                'stance_x': 0.0, 
                'stance_y': 0.0   
            },
        }

        self.PuppyPose = self.PP['LookDown_10deg'].copy()

        self.GaitConfig = {'overlap_time': 0.15, 'swing_time': 0.15, 'clearance_time': 0.0, 'z_clearance': 3.0}
        # Create publishers
        self.PuppyGaitConfigPub = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.PuppyVelocityPub = self.create_publisher(Velocity, '/puppy_control/velocity', 10)
        self.PuppyPosePub = self.create_publisher(Pose, '/puppy_control/pose', 10)

        # Create subscribers
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Create service clients
        self.runActionGroup_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup')
        
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.enable_running_srv = self.create_service(SetBool, '~/enable_running', self.enable_running_srv_callback)
        self.set_target_srv = self.create_service(SetString, '~/set_color_target', self.set_color_target_srv_callback)
        self.cli = self.create_client(Empty,'/puppy_control/go_home')

        # Wait for the service to be available
        while not self.runActionGroup_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /puppy_control/runActionGroup not available, waiting...')
   
    def initialize_robot(self):
        time.sleep(0.3)
        pose_msg = Pose()
        pose_msg.stance_x = float(self.PuppyPose['stance_x'])
        pose_msg.stance_y = float(self.PuppyPose['stance_y'])
        pose_msg.x_shift = float(self.PuppyPose['x_shift'])
        pose_msg.height = float(self.PuppyPose['height'])
        pose_msg.roll = float(self.PuppyPose['roll'])
        pose_msg.pitch = float(self.PuppyPose['pitch'])
        pose_msg.yaw = float(self.PuppyPose['yaw'])
        pose_msg.run_time = int(500)  # 确保是整数类型

        self.PuppyPosePub.publish(pose_msg)
        time.sleep(0.2)
        gait_msg = Gait()
        gait_msg.overlap_time = float(self.GaitConfig['overlap_time'])
        gait_msg.swing_time = float(self.GaitConfig['swing_time'])
        gait_msg.clearance_time = float(self.GaitConfig['clearance_time'])
        gait_msg.z_clearance = float(self.GaitConfig['z_clearance'])
        self.PuppyGaitConfigPub.publish(gait_msg)
        time.sleep(0.2)
        #threading.Thread(target=self.move).start()

    def enter_srv_callback(self, request, response):    
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter kick ball")
        self.initialize_robot()
        self.thread_started = False        
        time.sleep(0.02)
        
        response.success = True
        response.message = "start"
        return response
    
    
    def enable_running_srv_callback(self, request, response):
        self.enable_running = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % 'enable kick ball')
        
        response.success = True
        return response
        
    def set_color_target_srv_callback(self, request, response):
        #self.__target_color[0] = request.data
        self.__target_color = (request.data, )
        self.get_logger().info('\033[1;32mset color target %s\033[0m' % str(self.__target_color[0]))
        
        response.success = True
        return response
    
    
    def move(self):
        time.sleep(2)
        which_foot_kick_ball = 'left'  # Initialize
        self.is_shutdown = True

        while self.is_shutdown:
            if not self.is_shutdown:
                break
            if self.enable_running: 
                time.sleep(0.01)
                if self.puppyStatus == PuppyStatus.LOOKING_FOR:
                    if self.haved_detect:
                        self.puppyStatus = PuppyStatus.FOUND_TARGET
                        self.start_time = 0
                    else:
                        if self.start_time == 0:
                            self.start_time = time.time()                            
                        self.PuppyPose = self.PP['LookDown_10deg'].copy()
                        pose_msg = Pose()
                        pose_msg.stance_x = self.PuppyPose['stance_x']
                        pose_msg.stance_y = self.PuppyPose['stance_y']
                        pose_msg.x_shift = self.PuppyPose['x_shift']
                        pose_msg.height = self.PuppyPose['height']
                        pose_msg.roll = self.PuppyPose['roll']
                        pose_msg.pitch = self.PuppyPose['pitch']
                        pose_msg.yaw = self.PuppyPose['yaw']
                        self.PuppyPosePub.publish(pose_msg)
                        time.sleep(0.2)
                        time.sleep(0.8)
                        self.puppyStatus = PuppyStatus.LOOKING_FOR_LEFT
                elif self.puppyStatus == PuppyStatus.LOOKING_FOR_LEFT:
                    if self.haved_detect:
                        self.puppyStatus = PuppyStatus.FOUND_TARGET
                    else:
                        
                        velocity_msg = Velocity()
                        velocity_msg.x = float(3.0)
                        velocity_msg.y = float(0.0)
                        velocity_msg.yaw_rate = float(math.radians(-12))
                        self.PuppyVelocityPub.publish(velocity_msg)
    
                        time.sleep(3)
                        velocity_msg.yaw_rate = 0.0
                        self.PuppyVelocityPub.publish(velocity_msg)
                        time.sleep(0.3)
                        time.sleep(0.8)
                        self.puppyStatus = PuppyStatus.LOOKING_FOR_RIGHT
                elif self.puppyStatus == PuppyStatus.LOOKING_FOR_RIGHT:
                    if self.haved_detect:
                        self.start_time = 0
                        self.puppyStatus = PuppyStatus.FOUND_TARGET
                    else:
                        self.PuppyPose = self.PP['LookDown_10deg'].copy()
                        pose_msg = Pose()
                        pose_msg.stance_x = self.PuppyPose['stance_x']
                        pose_msg.stance_y = self.PuppyPose['stance_y']
                        pose_msg.x_shift = self.PuppyPose['x_shift']
                        pose_msg.height = self.PuppyPose['height']
                        pose_msg.roll = self.PuppyPose['roll']
                        pose_msg.pitch = self.PuppyPose['pitch']
                        pose_msg.yaw = self.PuppyPose['yaw']
                        self.PuppyPosePub.publish(pose_msg)
                        time.sleep(0.2)
                        velocity_msg = Velocity()
                        velocity_msg.x = 2.0
                        velocity_msg.y = 0.0
                        velocity_msg.yaw_rate = float(math.radians(-12))
                        self.PuppyVelocityPub.publish(velocity_msg)
                        self.elapsed_time = time.time() - self.start_time
                        #self.get_logger().info(f"self.elapsed_time:{self.elapsed_time}")
                        if self.elapsed_time > 3: 
                            self.start_time = 0
                            self.elapsed_time = 0
                            self.puppyStatus = PuppyStatus.LOOKING_FOR
                            velocity_msg = Velocity()
                            velocity_msg.x = 0.0
                            velocity_msg.y = 0.0
                            velocity_msg.yaw_rate = 0.0
                            self.PuppyVelocityPub.publish(velocity_msg)
                            self.haved_detect = False                    
                            self.enable_running = False
                            self.is_shutdown = False
                elif self.puppyStatus == PuppyStatus.FOUND_TARGET:
                    if self.target_info is None:
                        self.puppyStatus = PuppyStatus.LOOKING_FOR
                        continue
                    if self.target_info['centerY'] > 380:
                        self.puppyStatus = PuppyStatus.CLOSE_TO_TARGET
                        self.PuppyPose = self.PP['LookDown_20deg'].copy()
                        pose_msg = Pose()
                        pose_msg.stance_x = self.PuppyPose['stance_x']
                        pose_msg.stance_y = self.PuppyPose['stance_y']
                        pose_msg.x_shift = self.PuppyPose['x_shift']
                        pose_msg.height = self.PuppyPose['height']
                        pose_msg.roll = self.PuppyPose['roll']
                        pose_msg.pitch = self.PuppyPose['pitch']
                        pose_msg.yaw = self.PuppyPose['yaw']
                        self.PuppyPosePub.publish(pose_msg)
                        time.sleep(0.2)
                    else:
                        if self.expect_center['X'] - self.target_info['centerX'] < -80:
                            velocity_msg = Velocity()
                            velocity_msg.x = 3.0
                            velocity_msg.y = 0.0
                            velocity_msg.yaw_rate = float(math.radians(-12))
                            self.PuppyVelocityPub.publish(velocity_msg)
                            time.sleep(0.2)
                        elif self.expect_center['X'] - self.target_info['centerX'] > 80:
                            velocity_msg = Velocity()
                            velocity_msg.x = 3.0
                            velocity_msg.y = 0.0
                            velocity_msg.yaw_rate = float(math.radians(12))
                            self.PuppyVelocityPub.publish(velocity_msg)
                            time.sleep(0.2)
                        else:
                            velocity_msg = Velocity()
                            velocity_msg.x = 10.0
                            velocity_msg.y = 0.0
                            velocity_msg.yaw_rate = 0.0
                            self.PuppyVelocityPub.publish(velocity_msg)
                            time.sleep(0.2)
                elif self.puppyStatus == PuppyStatus.CLOSE_TO_TARGET:
                    if self.target_info is None:
                        self.puppyStatus = PuppyStatus.LOOKING_FOR
                        continue
                    if self.target_info['centerY'] > 380:
                        velocity_msg = Velocity()
                        velocity_msg.x = 0.0
                        velocity_msg.y = 0.0
                        velocity_msg.yaw_rate = 0.0
                        self.PuppyVelocityPub.publish(velocity_msg)
                        self.puppyStatus = PuppyStatus.CLOSE_TO_TARGET_FINE_TUNE
                        if self.expect_center['X'] > self.target_info['centerX']:
                            which_foot_kick_ball = 'left'
                        else:
                            which_foot_kick_ball = 'right'
                    else:
                        if self.expect_center['X'] - self.target_info['centerX'] < -50:
                            velocity_msg = Velocity()
                            velocity_msg.x = 4.0
                            velocity_msg.y = 0.0
                            velocity_msg.yaw_rate = float(math.radians(-8))
                            self.PuppyVelocityPub.publish(velocity_msg)
                            time.sleep(0.2)
                        elif self.expect_center['X'] - self.target_info['centerX'] > 50:
                            velocity_msg = Velocity()
                            velocity_msg.x = 4.0
                            velocity_msg.y = 0.0
                            velocity_msg.yaw_rate = float(math.radians(8))
                            self.PuppyVelocityPub.publish(velocity_msg)
                            time.sleep(0.2)
                        else:
                            velocity_msg = Velocity()
                            velocity_msg.x = 8.0
                            velocity_msg.y = 0.0
                            velocity_msg.yaw_rate = 0.0
                            self.PuppyVelocityPub.publish(velocity_msg)
                            time.sleep(0.2)
                elif self.puppyStatus == PuppyStatus.CLOSE_TO_TARGET_FINE_TUNE:
                    if self.target_info is None:
                        self.puppyStatus = PuppyStatus.LOOKING_FOR
                        continue
                    if self.target_info['centerY'] < self.expect_center_kick_ball_left['Y']:
                        velocity_msg = Velocity()
                        velocity_msg.x = 4.0
                        velocity_msg.y = 0.0
                        velocity_msg.yaw_rate = 0.0
                        self.PuppyVelocityPub.publish(velocity_msg)
                        time.sleep(0.1)
                    elif which_foot_kick_ball == 'left' and self.target_info['centerX'] > self.expect_center_kick_ball_left['X']:
                        velocity_msg = Velocity()
                        velocity_msg.x = 0.0
                        velocity_msg.y = 0.0
                        velocity_msg.yaw_rate = float(math.radians(-5))
                        self.PuppyVelocityPub.publish(velocity_msg)
                        time.sleep(0.1)
                    elif which_foot_kick_ball == 'right' and self.target_info['centerX'] < self.expect_center_kick_ball_right['X']:
                        velocity_msg = Velocity()
                        velocity_msg.x = 0.0
                        velocity_msg.y = 0.0
                        velocity_msg.yaw_rate = float(math.radians(5))
                        self.PuppyVelocityPub.publish(velocity_msg)
                        time.sleep(0.1)
                    else:
                        velocity_msg = Velocity()
                        velocity_msg.x = 5.0
                        velocity_msg.y = 0.0
                        if which_foot_kick_ball == 'left':
                            velocity_msg.yaw_rate = float(math.radians(-8))
                        else:
                            velocity_msg.yaw_rate = float(math.radians(8))
                        self.PuppyVelocityPub.publish(velocity_msg)
                        time.sleep(1.66)
                        velocity_msg.x = 0.0
                        velocity_msg.y = 0.0
                        velocity_msg.yaw_rate = 0.0
                        self.PuppyVelocityPub.publish(velocity_msg)
                        self.puppyStatus = PuppyStatus.KICK_BALL
                elif self.puppyStatus == PuppyStatus.KICK_BALL:
                    velocity_msg = Velocity()
                    velocity_msg.x = 0.0
                    velocity_msg.y = 0.0
                    velocity_msg.yaw_rate = 0.0
                    self.PuppyVelocityPub.publish(velocity_msg)
                    time.sleep(0.2)
                    request = SetRunActionName.Request()
                    if which_foot_kick_ball == 'left':
                        request.name = 'kick_ball_left.d6ac'
                    else:
                        request.name = 'kick_ball_right.d6ac'
                    request.wait = True
                    future = self.runActionGroup_srv.call_async(request)                    
                    future.add_done_callback(self.kick_ball_callback)
                    self.puppyStatus = PuppyStatus.LOOKING_FOR                   
                    self.haved_detect = False                    
                    self.enable_running = False
                    self.is_shutdown = False
                    self.puppyStatus = PuppyStatus.LOOKING_FOR
                    time.sleep(2)
                    self.cli.call_async(Empty.Request())
                    time.sleep(1)
                elif self.puppyStatus == PuppyStatus.STOP:
                    velocity_msg = Velocity()
                    velocity_msg.x = 0.0
                    velocity_msg.y = 0.0
                    velocity_msg.yaw_rate = 0.0
                    self.PuppyVelocityPub.publish(velocity_msg)
                else:
                    time.sleep(0.01)
            
                if self.puppyStatusLast != self.puppyStatus:
                    self.get_logger().info(f'puppyStatus: {self.puppyStatus}')
                self.puppyStatusLast = self.puppyStatus
            else:
                time.sleep(0.02)
    def kick_ball_callback(self, future):
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Kick ball action completed successfully.')
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))
        # 状态已经在 move 方法中重置为 LOOKING_FOR

    def run(self, img):
        img_h, img_w = img.shape[:2]
        frame_resize = cv2.resize(img, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)
        max_area = 0
        color_area_max = None
        areaMaxContour_max = None
        for i in self.color_range_list:
            if i in self.__target_color:
                color_range = self.color_range_list[i]
                frame_mask = cv2.inRange(frame_lab,
                                         np.array(color_range['min'], dtype=np.uint8),
                                         np.array(color_range['max'], dtype=np.uint8))
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                if self.debug:
                    cv2.imshow(i, dilated)
                contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                areaMaxContour, area_max = self.getAreaMaxContour(contours)
                if areaMaxContour is not None:
                    if area_max > max_area:
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 200:
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            centerX = int(map_value(rect[0][0], 0, self.size[0], 0, img_w))
            centerY = int(map_value(rect[0][1], 0, self.size[1], 0, img_h))
            sideX = int(map_value(rect[1][0], 0, self.size[0], 0, img_w))
            sideY = int(map_value(rect[1][1], 0, self.size[1], 0, img_h))
            angle = rect[2]
            for i in range(4):
                box[i, 1] = int(map_value(box[i, 1], 0, self.size[1], 0, img_h))
            for i in range(4):
                box[i, 0] = int(map_value(box[i, 0], 0, self.size[0], 0, img_w))
            cv2.drawContours(img, [box], -1, (0, 0, 255), 2)
            if color_area_max == 'red':
                color = 1
            elif color_area_max == 'green':
                color = 2
            elif color_area_max == 'blue':
                color = 3
            else:
                color = 0
            self.color_list.append(color)
            if len(self.color_list) == 3:
                color = int(round(np.mean(np.array(self.color_list))))
                self.color_list = []
                if color == 1:
                    self.detect_color = 'red'
                    self.draw_color = self.range_rgb["red"]
                elif color == 2:
                    self.detect_color = 'green'
                    self.draw_color = self.range_rgb["green"]
                elif color == 3:
                    self.detect_color = 'blue'
                    self.draw_color = self.range_rgb["blue"]
                else:
                    self.detect_color = 'None'
                    self.draw_color = self.range_rgb["black"]
        else:
            self.detect_color = 'None'
            self.draw_color = self.range_rgb["black"]
        if self.detect_color == self.__target_color[0]:
            self.haved_detect = True
            if sideX > sideY:
                self.target_info = {'centerX': centerX, 'centerY': centerY, 'sideX': sideX, 'sideY': sideY, 'scale': sideX/sideY, 'angle': angle}
            else:
                self.target_info = {'centerX': centerX, 'centerY': centerY, 'sideX': sideX, 'sideY': sideY, 'scale': sideY/sideX, 'angle': angle}
        else:
            self.haved_detect = False
        cv2.putText(img, "Color: " + self.detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, self.draw_color, 2)
        return img

    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None
        for c in contours:
            contour_area_temp = math.fabs(cv2.contourArea(c))
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp >= 5:
                    area_max_contour = c
        return area_max_contour, contour_area_max

    def image_callback(self, ros_image):
        cv2_img = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        frame = cv2_img.copy()
        frame_result = frame
        
        if not self.thread_started:
            self.thread = threading.Thread(target=self.move)
            self.thread.start()
            self.thread_started = True 
        
        if self.enable_running:             
            frame_result = self.run(frame)                        
            cv2.imshow('Frame', frame_result)           
            cv2.waitKey(1)
            
        else:
            
            cv2.destroyAllWindows()

    def destroy_node(self):
        self.is_shutdown = True
        velocity_msg = Velocity()
        velocity_msg.x = 0.0
        velocity_msg.y = 0.0
        velocity_msg.yaw_rate = 0.0
        self.PuppyVelocityPub.publish(velocity_msg)
        super().destroy_node()
        rclpy.shutdown()
        #sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    kick_ball_demo = KickBallDemo()
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(kick_ball_demo)
        executor.spin()
        #rclpy.spin(kick_ball_demo)
    except KeyboardInterrupt:
        pass
    finally:
        #kick_ball_demo.destroy_node()
        kick_ball_demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
