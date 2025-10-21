#!/usr/bin/python3
# coding=utf8
# Date:2024/090/13
# Author:liuyuan
# 第7课 标签坐标定位实验/第8课 标签追踪实验
# 运行程序时通过添加参数来切换功能--model 0/1(默认是0)
import sys
import rclpy
from rclpy.node import Node
import cv2
import math
import threading
import numpy as np
import apriltag
import signal
import queue
import argparse
import time
from threading import RLock, Timer
from std_srvs.srv import *
from threading import RLock, Timer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from puppy_control_msgs.msg import Velocity, Pose, Gait

class AprilTagTrackingDemo(Node):
    def __init__(self):
        super().__init__('apriltag_tracking')
        
        # 声明变量
        self.lock = RLock()
        self.__isRunning = False
        self.org_image_sub_ed = False
        self.times = 0  
        self.tag_id = None
        self.coordinate = None # 标签距离摄像头的坐标[x，y，z]，单位米(coordinates of the tag relative to the camera [x, y, z], measured in meters)
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        self.haved_detect = False
        self.detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
        self.camera_intrinsic = np.matrix([  [619.063979, 0,          302.560920],
                                        [0,          613.745352, 237.714934],
                                        [0,          0,          1]])
                
        signal.signal(signal.SIGINT, self.stop)
        self.pose_publisher = self.create_publisher(Pose, '/puppy_control/pose',10)
        self.gait_publisher = self.create_publisher(Gait, '/puppy_control/gait', 10)
        self.velocity_publisher = self.create_publisher(Velocity, '/puppy_control/velocity',10)
        self.puppy_set_running_srv = self.create_client(SetBool,'/puppy_control/set_running' )
        # 通过参数来选择模式
        if self.parse_opt().model == 1:
            # 运行子线程(run sub-thread)
            th = threading.Thread(target=self.move,daemon=True)
            th.start()
  
        # 进入主程序
        self.debug = True
        if self.debug:           
            self.enter_func(1)     
            self.start_running()             
            
    def enter_func(self, msg):
        self.get_logger().info("enter apriltag tracking") 
        self.init()
        with self.lock:
            if not self.org_image_sub_ed:
                self.org_image_sub_ed = True
                
                self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)# register result image publisher
                
        return [True, 'enter']
        
    def exit_func(self,msg):
        self.get_logger().info("exit apriltag tracking") 
        with self.lock:
            self.__isRunning = False
            reset()
            try:
                if self.org_image_sub_ed:
                    self.org_image_sub_ed = False
            except BaseException as e:
                self.get_logger().info(e) 
        
    
        return [True, 'exit']
        
    def start_running(self,):
        self.get_logger().info("start running apriltag tracking")
        with self.lock:
            self.__isRunning = True
    
    # 运动控制
    def move(self,):
        while True:
            if self.__isRunning:
                if self.coordinate is None:
                    self.set_move()
                    time.sleep(0.01)   
                else:
                    if self.coordinate[2] > 0.22:
                        self.set_move(x = 5.0)                            
                    elif self.coordinate[2] < 0.18:
                        self.set_move(x = -5.0)
                    else:
                        self.set_move(x=0.0, y=0.0, yaw_rate=0.0)
                time.sleep(0.01)         
            else:
                time.sleep(0.01)
    
    
    # 初始化步态和姿态    
    def init(self,):
        self.get_logger().info("apriltag detect init")
        self.set_pose()
        self.set_gait()
        
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
        cv2.imshow('image', frame_result)
        cv2.waitKey(1)
        
    # 检测标签
    def run(self,img):
        if not self.__isRunning:
            return img
        
        self.tag_id = self.apriltagDetect(img) # apriltag检测(apriltag detection)
        if self.tag_id is not None and not self.haved_detect:
            self.haved_detect = True
        cv2.putText(img, self.tag_id, (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)   
        
        return img
        
    # 标签处理
    def apriltagDetect(self,img):              
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        detections = self.detector.detect(gray, return_image=False)
        if len(detections) != 0:
            for detection in detections:
                M,e0,e1 = self.detector.detection_pose(detection,[self.camera_intrinsic.item(0,0), self.camera_intrinsic.item(1,1),
                                                                    self.camera_intrinsic.item(0,2), self.camera_intrinsic.item(1,2)],
                                                                    0.033)  
                P = M[:3,:4]
                self.coordinate=np.matmul(P,np.array([[0],[0],[0],[1]])).flatten()
                print('coordinate = ',self.coordinate)    
                tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family(get tag_family)
                self.times = 0
                if tag_family == 'tag36h11':
                    tag_id = str(detection.tag_id)  # 获取tag_id(get tag_id)
                    return tag_id
                else:
                    return None                    
        else:
            self.times += 1
            time.sleep(1)
            if self.times >= 3:
                self.coordinate = None
            return None
    # 退出程序 
    #def stop(self):
        #self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0)) 
      #  self.set_move()
        #self.get_logger().info("stop running apriltag tracking")
    def stop(self,signum, frame):  
        
        self.velocity_publisher.publish(Velocity(x=0.0, y=0.0, yaw_rate=0.0))
        self.get_logger().info('Shutting down...')
        sys.exit()

    # 设置姿态
    def set_pose(self,roll=math.radians(0), pitch=math.radians(0), yaw=0.000, height=-10.0, x_shift=0.5, stance_x= 0.0, stance_y=0.0,run_time=500):
        self.pose_publisher.publish(Pose(stance_x=stance_x, stance_y=stance_y, x_shift=x_shift,
                                       height=height, roll=roll, pitch=pitch, yaw=yaw, run_time=500))
    # 设置步态                                   
    def set_gait(self,overlap_time = 0.2, swing_time = 0.15, clearance_time = 0.0, z_clearance = 4.0):      
        self.gait_publisher.publish(Gait(overlap_time = overlap_time, swing_time = swing_time, clearance_time =clearance_time, z_clearance = z_clearance))
    # 运动控制    
    def set_move(self, x=0.00, y=0.0, yaw_rate=0.0):
        self.velocity_publisher.publish(Velocity(x=x, y=y, yaw_rate=yaw_rate)) 
        
    def parse_opt(self,):
        parser = argparse.ArgumentParser()
        parser.add_argument('--model', type=int, default=0, help='0 or 1')
        opt = parser.parse_args()
        return opt

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTrackingDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
