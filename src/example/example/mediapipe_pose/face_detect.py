#!/usr/bin/env python3
# encoding: utf-8
import cv2
import queue
import rclpy
import threading
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class SegmentationNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        
        self.face = mp.solutions.face_detection
        self.face_detection = self.face.FaceDetection(min_detection_confidence=0.6)
        self.img_h, self.img_w = 0, 0
        self.bridge = CvBridge()
        
        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):                               
        rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
                               
        img_copy = rgb_image.copy()
        self.img_h, self.img_w = rgb_image.shape[:2]
        imgRGB = cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB)
        results = self.face_detection.process(imgRGB)
        if results.detections:
            for index, detection in enumerate(results.detections):
                bboxC = detection.location_data.relative_bounding_box
                bbox = (int(bboxC.xmin * self.img_w), int(bboxC.ymin * self.img_h),int(bboxC.width * self.img_w), int(bboxC.height * self.img_h))
                cv2.rectangle(rgb_image, bbox, (0,255,0), 2)
                
                
        if self.image_queue.full():
            # 濡傛灉闃熷垪宸叉弧锛屼涪寮冩渶鏃х殑鍥惧儚(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 灏嗗浘鍍忔斁鍏ラ槦鍒?put the image into the queue)
        self.image_queue.put(rgb_image)


    def main(self):
    
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 1)
            cv2.imshow('MediaPipe Face Detect', image)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            key = cv2.waitKey(1)
            if key == ord('q') or key == 27:  # 鎸塹鎴栬€卐sc閫€鍑?press q or esc to exit)
                break
            else: 
                pass
        cv2.destroyAllWindows()
        rclpy.shutdown()
            
    

def main():
    node = SegmentationNode('self_segmentation')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')


if __name__ == "__main__":
  main()
