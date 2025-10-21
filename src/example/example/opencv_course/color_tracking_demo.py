#!/usr/bin/python3
# coding=utf8
# Date:2024/090/13
# Author:liuyuan

# 第5课 色块定位实验/第6课 颜色追踪实验
# 运行程序时通过添加参数来切换功能--model 0/1(默认是0)
import sys
import cv2
import math
import rclpy
from rclpy.node import Node
import numpy as np
from threading import RLock, Timer
import sdk.common as common
import time
import queue
import signal
import sdk.PID as pid
import argparse
from std_srvs.srv import *
from sensor_msgs.msg import Image
from ros_robot_controller_msgs.msg import RGBState, RGBsState
from cv_bridge import CvBridge
from puppy_control_msgs.srv import SetRunActionName
from puppy_control_msgs.msg import Velocity, Pose, Gait


class ColorTrackingNode(Node):
    def __init__(self):
        super().__init__('color_tracking')
        signal.signal(signal.SIGINT, self.stop)
        # 声明变量
        self.lock = RLock()
        self.range_rgb = {
                          'red': (0, 0, 255),
                          'blue': (255, 0, 0),
                          'green': (0, 255, 0),
                          'black': (0, 0, 0),
                          'white': (255, 255, 255),}
        self.color_range = None
        #self.x_pid = pid.PID(P=0.003, I=0.00, D=0.00)  # pid初始化(pid initialization)
        #self.y_pid = pid.PID(P=0.00001, I=0.0000, D=0.0000)
        #self.z_pid = pid.PID(P=0.0015, I=0.0000, D=0.0000)
        
        self.x_pid = pid.PID(P=0.06, I=0.0005, D=0.00)  # pid初始化(pid initialization)
        self.y_pid = pid.PID(P=0.00001, I=0.0000, D=0.0000)
        self.z_pid = pid.PID(P=0.003, I=0.0000, D=0.0000)

        self.__isRunning = False
        self.start_move = True
        self.image_sub = None
        self.__target_color = None
        self.org_image_sub_ed = False
        self.bridge = CvBridge()
        self.size = (320, 240)
        self.image_queue = queue.Queue(2)
        
        
        self.pose_publisher = self.create_publisher(Pose, '/puppy_control/pose',10)
        self.gait_publisher = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.velocity_publisher = self.create_publisher(Velocity, '/puppy_control/velocity',10)
        self.rgb_pub = self.create_publisher(RGBsState,'/ros_robot_controller/set_rgb',10)
        self.run_action_group_srv = self.create_client(SetRunActionName, '/puppy_control/runActionGroup',)
        self.puppy_set_running_srv = self.create_client(SetBool,'/puppy_control/set_running' )
        
        self.debug = True
        
        # 进入主程序
        if self.debug:
            self.set_target('red')
            self.enter_func(1)     
            
            self.start_running()    
    
    # 控制RGB
    def set_rgb(self,r, g, b):
        rgb1 = RGBState()
        rgb1.id = 1
        rgb1.r = r
        rgb1.g = g
        rgb1.b = b
        rgb2 = RGBState()
        rgb2.id = 2
        rgb2.r = r
        rgb2.g = g
        rgb2.b = b
        msg = RGBsState()
        msg.data = [rgb1, rgb2]
        self.rgb_pub.publish(msg)
        
    def enter_func(self, msg):
        self.get_logger().info("enter object tracking") 
        self.init()
        with self.lock:
            if not self.org_image_sub_ed:
                self.org_image_sub_ed = True
                self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)# register result image publisher
                
        #puppy_set_running_srv(True)
        return [True, 'enter']
        
    def exit_func(self,msg):
        self.get_logger().info("exit object tracking") 
        with self.lock:
            self.__isRunning = False
            reset()
            try:
                if self.org_image_sub_ed:
                    self.org_image_sub_ed = False
            except BaseException as e:
                self.get_logger().info(e) 
        
    
        return [True, 'exit']
    # 初始化    
    def init(self):
        # 执行站立动作组
        msg = SetRunActionName.Request()
        msg.name = 'sit.d6ac'
        msg.wait = True
        self.run_action_group_srv.call_async(msg)
        
        self.color_range = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")# get lab range from ros param server
        self.get_logger().info("object tracking Init")  
    
        
    def start_running(self,):
        self.get_logger().info("start running object tracking")
        with self.lock:
            self.__isRunning = True

    def stop_running(self,):
        self.get_logger().info("stop running object tracking")
        with self.lock:
            self.__isRunning = False
    
    def set_running(self,msg):
        if msg.data:
            self.start_running()
        else:
            self.stop_running()  
        return [True, 'set_running']
    
    # 设置目标颜色
    def set_target(self,color):
        with self.lock:
            self.__target_color = color
            self.set_rgb(self.range_rgb[self.__target_color][2],self.range_rgb[self.__target_color][1],self.range_rgb[self.__target_color][0])
            
        
    # 退出程序，关闭RGB    
    def stop(self):
        self.set_rgb(0, 0, 0)
        self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0)) 
        print('is_shutdown')
        
    # 图像的回调函数    
    def image_callback(self,ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)
        image = self.image_queue.get(block=True, timeout=1)      
        frame = image.copy()
        frame_result = frame
        with self.lock:
            if self.__isRunning:
                frame_result = self.run(frame)
                cv2.imshow('Frame', frame_result)
                key = cv2.waitKey(1)
    # 主程序    
    def run(self,img):
        global PuppyMove
        global x_dis, y_dis, z_dis
        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
    
        cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)), (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
        cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)), (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)
    
        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    
        area_max = 0
        area_max_contour = 0
        
        if self.__target_color in list(self.color_range['color_range_list'].keys()):   
            target_color_range = self.color_range['color_range_list'][self.__target_color]
            frame_mask = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(target_color_range['max']))  # 对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrosion)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilation)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find out the contour)
            area_max_contour, area_max = common.get_area_max_contour(contours, 500)  # 找出最大轮廓(find out the contour with the maximal area)
    
        if area_max > 100:  # 有找到最大面积(the maximal area is found)
        
            if area_max_contour is not None and len(area_max_contour) > 0:        
                (center_x, center_y), radius = cv2.minEnclosingCircle(area_max_contour)  # 获取最小外接圆(get the minimum circumcircle)
            else:
                 return img
            
            center_x = int(common.val_map(center_x, 0, self.size[0], 0, img_w))
            center_y = int(common.val_map(center_y, 0, self.size[1], 0, img_h))
            radius = int(common.val_map(radius, 0, self.size[0], 0, img_w))
            if radius > 100:
                return img
            print('center_x = %d ,center_y=%d'%(int(center_x), int(center_y)))
            cv2.circle(img, (int(center_x), int(center_y)), int(radius), (0,0,255), 2)
            if self.start_move and self.parse_opt().model == 1: 
                       
                self.x_pid.Kp = 0.003
                self.x_pid.Ki = 0.00
                self.x_pid.Kd = 0.00
                
                self.x_pid.SetPoint = img_w / 2.0  # 设定(set)
                if abs(self.x_pid.SetPoint - center_x) > 230:
                    self.x_pid.Kp = 0.004    
                self.x_pid.update(center_x)
                self.x_dis = self.x_pid.output    
                self.x_dis = np.radians(30) if self.x_dis > np.radians(30) else self.x_dis
                self.x_dis = np.radians(-30) if self.x_dis < np.radians(-30) else self.x_dis
                
                
                self.z_pid.Kp = 0.0015
                self.z_pid.Ki = 0.0000
                self.z_pid.Kd = 0.0000
                
                self.z_pid.SetPoint = img_h / 2.0
                if abs(self.z_pid.SetPoint - center_y) > 180:
                    self.z_pid.Kp = 0.002
                self.z_pid.update(center_y)
                self.z_dis = self.z_pid.output
                self.z_dis = np.radians(30) if self.z_dis > np.radians(30) else self.z_dis
                self.z_dis = np.radians(-20) if self.z_dis < np.radians(-20) else self.z_dis
                
                
                if abs(area_max - 900) < 150:
                    self.y_dis = 0.0
                elif area_max - 900 < -150:
                    self.y_dis = 10.0
                elif area_max - 900 > 150:
                    self.y_dis = -7.0  
               #self.velocity_publisher.publish(Velocity(x=self.y_dis, y=0.0, yaw_rate=0.0)) 
                self.set_pose(roll = self.x_dis,pitch=self.z_dis,run_time = 30)
                print("x_dis",self.y_dis)
        return img    
    
    def set_pose(self,roll=math.radians(0), pitch=math.radians(0), yaw=0.000, height=-10.0, x_shift=0.5, stance_x= 0.0, stance_y=0.0,run_time=500): 
        self.pose_publisher.publish(Pose(stance_x=stance_x, stance_y=stance_y, x_shift=x_shift,
                                           height=height, roll=roll, pitch=pitch, yaw=yaw, run_time=500))
    def parse_opt(self,):
        parser = argparse.ArgumentParser()
        parser.add_argument('--model', type=int, default=0, help='0 or 1')
        opt = parser.parse_args()
        return opt   

def main(args=None):
    rclpy.init(args=args)
    color_tracker = ColorTrackingNode()
    
    try:
        rclpy.spin(color_tracker)
    except KeyboardInterrupt:
        print("intteruprt------------")
        pass
    finally:
        color_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


