#!/usr/bin/python3
# coding=utf8
# 第11章 ROS机器狗结合机械臂课程\第6课 手势控制机械臂(11.ROS Robot Dog Combine Robotic Arm\Lesson 6 Posture Control Robotic Arm)
import sys
import cv2
import math
import time
import rospy
import mediapipe as mp
from collections import deque
import threading
import numpy as np
from common import Misc
from common import PID
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from arm_kinematics.ArmMoveIK import ArmIK
from std_srvs.srv import *

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse
# from puppy_control.srv import SetRunActionName

def distance(point_1, point_2):
    """
    计算两个点间的距离(calculate the distance between two points)
    :param point_1: 点1(point 1)
    :param point_2: 点2(point 2)
    :return: 两点间的距离(distance between two points)
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)

def get_hand_landmarks(img, landmarks):
    """
    将landmarks从medipipe的归一化输出转为像素坐标(convert the landmarks from mediapipe's normalized output to pixel coordinates)
    :param img: 像素坐标对应的图片(the image corresponding to the pixel coordinates)
    :param landmarks: 归一化的关键点(normalized key points)
    :return:
    """
    h, w, _ = img.shape
    landmarks = [(lm.x * w, lm.y * h) for lm in landmarks]
    return np.array(landmarks)

def cv2_image2ros(image, frame_id=''):
    image = image[:,:,::-1]
    ros_image = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = frame_id
    ros_image.height = image.shape[:2][0]
    ros_image.width = image.shape[:2][1]
    ros_image.encoding = 'rgb8'
    ros_image.data = image.tostring()
    ros_image.header = header
    ros_image.step = ros_image.width * 3

    return ros_image

class HandControlWithArmNode:
    def __init__(self, name):
        rospy.init_node(name)  # launch里的name会覆盖此处的name，所以要修改name，需要修改launch里的name, 为了一致性此处name会和launch name保持一致(the "name" in the "launch" file will override the name here. To change "name", you need to modify it in the "launch" file. For consistency, the name here will match the name in the launch file)
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
        
        self.window_size = 5  # 定义滑动窗口大小（滤波窗口大小）(define the size of slider window(filter window size))
        self.distance_measurements = deque(maxlen=self.window_size) # 创建一个队列用于存储最近的距离测量数据(create a queue to store recent distance measurement data)
        
        self.filtered_distance = 0
        self.th = threading.Thread(target=self.move,daemon=True)
        self.th.start()
        self.name = name
        self.image = None
        
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布,# 使用~可以自动加上前缀名称(image processing result publish, use "~" can automatically add the prefix name)
    
    def image_callback(self, ros_image):
        self.image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)
    
    def init(self):
        setServoPulse(9,1500,300)
        setServoPulse(10,800,300)
        setServoPulse(11,850,300)
        time.sleep(0.3)
        
        
    
    def move(self):
        while not rospy.is_shutdown():
            
            if self.filtered_distance > 0 and self.filtered_distance < 100:
                
                
                out = Misc.map(self.filtered_distance, 15, 90, 1500, 900)
                setServoPulse(9,int(out),0)
 
            else:
                rospy.sleep(0.001)

    def run(self):
        
        self.init()
        while not rospy.is_shutdown():
            if self.image is not None:
                image_flip = cv2.flip(self.image, 1)
                image_re = cv2.resize(image_flip, (320,240), interpolation=cv2.INTER_NEAREST)
                self.image = None
                bgr_image = cv2.cvtColor(image_re, cv2.COLOR_RGB2BGR)
                try:
                    
                       
                    results = self.hands.process(image_re)
                    if results.multi_hand_landmarks:
                        
                        for hand_landmarks in results.multi_hand_landmarks:
                            ## mediapipe中首部关键结点的连线(lines connecting the key points at the beginning of the MediaPipe)
                            self.mpDraw.draw_landmarks(bgr_image, hand_landmarks, self.mpHands.HAND_CONNECTIONS)
                            landmarks = get_hand_landmarks(image_re, hand_landmarks.landmark)
                            
                            cv2.line(bgr_image, (int(landmarks[4][0]), int(landmarks[4][1])), (int(landmarks[8][0]), int(landmarks[8][1])), (0, 255, 255), 2)
                            cv2.circle(bgr_image, (int(landmarks[8][0]), int(landmarks[8][1])), 4, (0, 255, 255), -1)
                            cv2.circle(bgr_image, (int(landmarks[4][0]), int(landmarks[4][1])), 4, (0, 255, 255), -1)

                            distanceSum = distance(landmarks[8], landmarks[4])
                           
                            distanceSum = max(15, min(distanceSum, 90))
                            self.distance_measurements.append(distanceSum)
                            self.filtered_distance = np.mean(self.distance_measurements)
                            self.filtered_distance = round(self.filtered_distance,2)
                            cv2.putText(bgr_image,'DST: '+str(self.filtered_distance), (5,220), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
                                
                                
                    self.frames +=1
                    delta_time = time.time() - self.last_time
                    cur_fps = np.around(self.frames / delta_time, 1)
                    cv2.putText(bgr_image,'FPS: '+str(cur_fps), (5,30), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
                except Exception as e:
                    print(e)
              
                
                self.result_publisher.publish(cv2_image2ros(cv2.resize(bgr_image, (640, 480)), self.name))
                                               

if __name__ == '__main__':
    HandControlWithArmNode('hand_control').run()

