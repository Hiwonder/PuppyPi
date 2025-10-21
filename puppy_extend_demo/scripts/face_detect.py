#!/usr/bin/env python
# encoding: utf-8
import cv2
import queue
import rospy
import threading
import numpy as np
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class FaceDetect:
    def __init__(self):
        rospy.init_node('face_detect_demo', anonymous=True)
        self.running = True
        self.bridge = CvBridge()  # ROS1需要cv_bridge进行图像转换
        
        # Mediapipe初始化
        self.face = mp.solutions.face_detection
        self.face_detection = self.face.FaceDetection(min_detection_confidence=0.6)
        self.img_h, self.img_w = 0, 0
        
        # 图像队列和订阅者
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
        rospy.loginfo('\033[1;32m%s\033[0m' % 'start')
        
        # 启动处理线程
        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        try:
            # 使用cv_bridge转换ROS图像消息
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except Exception as e:
            rospy.logerr(e)
            return
        
        img_copy = cv_image.copy()
        self.img_h, self.img_w = cv_image.shape[:2]
        
        # Mediapipe处理
        imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)
        results = self.face_detection.process(imgRGB)
        
        if results.detections:
            for detection in results.detections:
                bboxC = detection.location_data.relative_bounding_box
                bbox = (int(bboxC.xmin * self.img_w), 
                        int(bboxC.ymin * self.img_h),
                        int(bboxC.width * self.img_w), 
                        int(bboxC.height * self.img_h))
                cv2.rectangle(cv_image, bbox, (0,255,0), 2)
        
        # 放入队列
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(cv_image)

    def main(self):
        while self.running and not rospy.is_shutdown():
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue
            
            # 显示处理
            display_image = cv2.flip(image, 1)
            cv2.imshow('MediaPipe Face Detect', display_image)
            
            key = cv2.waitKey(1)
            if key in [ord('q'), 27]:  # ESC或q键退出
                self.running = False
                break
        
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User requested shutdown")

if __name__ == "__main__":
    try:
        node = FaceDetect()
        rospy.spin()  # ROS1需要主动调用spin
    except Exception as e:
        rospy.logerr(e)
    finally:
        cv2.destroyAllWindows()
