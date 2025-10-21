#!/usr/bin/python3
# coding=utf8
# Date:2024/09/03
# Author:liuyuan
# 标签识别
#!/usr/bin/python3
# coding=utf8
# Date:2024/09/03
# Author:liuyuan
# 标签识别
import cv2
import math
import yaml
import queue
import rclpy
import threading
import numpy as np
import apriltag
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
coordinate = None

tag_id = None
haved_detect = False


detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
camera_intrinsic = np.matrix([  [619.063979, 0,          302.560920],
                                [0,          613.745352, 237.714934],
                                [0,          0,          1]])
                             
times = 0
def apriltagDetect(img):  
    global times
    global coordinate

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray, return_image=False)
    if len(detections) != 0:
        for detection in detections:
            M,e0,e1 = detector.detection_pose(detection,[camera_intrinsic.item(0,0), camera_intrinsic.item(1,1),
                                                                camera_intrinsic.item(0,2), camera_intrinsic.item(1,2)],
                                                                0.033)                
            P = M[:3,:4]
            coordinate=np.matmul(P,np.array([[0],[0],[0],[1]])).flatten()
            print('coordinate =',coordinate)    

            corners = np.rint(detection.corners)  # 获取四个角点(get for corners)
            cv2.drawContours(img, [np.array(corners, int)], -1, (0, 255, 255), 5, cv2.LINE_AA)
            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family(get tag_family)
            times = 0
            if tag_family == 'tag36h11':
                tag_id = str(detection.tag_id)  # 获取tag_id(get tag_id)
                print('tag_id =',tag_id)
                return tag_id
            else:
                return None
            
    else:
        times += 1
        if times >= 3:
            coordinate = None
        return None

def run(img):
    global tag_id
    global haved_detect

    tag_id = apriltagDetect(img) # apriltag检测(apriltag detection)
    if tag_id is not None and not haved_detect:
        haved_detect = True
    cv2.putText(img, tag_id, (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 1)
    return img

ROS_NODE_NAME = 'apriltag_detect_demo'
class ApriltagNode(Node):
    def __init__(self):
        super().__init__(ROS_NODE_NAME)
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, f'/{ROS_NODE_NAME}/image_result', 1)
        self.image_queue = queue.Queue(2)
    def image_callback(self, ros_image):
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)
        cv2_img = self.image_queue.get(block=True, timeout=1)
        #cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        frame = cv2_img.copy()
        frame_result = frame

        frame_result = run(frame)
        cv2.imshow('image', frame_result)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    apriltag_node = ApriltagNode()
    rclpy.spin(apriltag_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
        
        
import cv2
import math
import yaml
import queue
import rclpy
import threading
import numpy as np
import apriltag
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
coordinate = None

tag_id = None
haved_detect = False


detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
camera_intrinsic = np.matrix([  [619.063979, 0,          302.560920],
                                [0,          613.745352, 237.714934],
                                [0,          0,          1]])
                             
times = 0
def apriltagDetect(img):  
    global times
    global coordinate

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray, return_image=False)
    if len(detections) != 0:
        for detection in detections:
            M,e0,e1 = detector.detection_pose(detection,[camera_intrinsic.item(0,0), camera_intrinsic.item(1,1),
                                                                camera_intrinsic.item(0,2), camera_intrinsic.item(1,2)],
                                                                0.033)                
            P = M[:3,:4]
            coordinate=np.matmul(P,np.array([[0],[0],[0],[1]])).flatten()
            print('coordinate =',coordinate)    

            corners = np.rint(detection.corners)  # 获取四个角点(get for corners)
            cv2.drawContours(img, [np.array(corners, int)], -1, (0, 255, 255), 5, cv2.LINE_AA)
            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family(get tag_family)
            times = 0
            if tag_family == 'tag36h11':
                tag_id = str(detection.tag_id)  # 获取tag_id(get tag_id)
                print('tag_id =',tag_id)
                return tag_id
            else:
                return None
            
    else:
        times += 1
        if times >= 3:
            coordinate = None
        return None

def run(img):
    global tag_id
    global haved_detect

    tag_id = apriltagDetect(img) # apriltag检测(apriltag detection)
    if tag_id is not None and not haved_detect:
        haved_detect = True
    cv2.putText(img, tag_id, (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 1)
    return img

ROS_NODE_NAME = 'apriltag_detect_demo'
class ApriltagNode(Node):
    def __init__(self):
        super().__init__(ROS_NODE_NAME)
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, f'/{ROS_NODE_NAME}/image_result', 1)
        self.image_queue = queue.Queue(2)
    def image_callback(self, ros_image):
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(bgr_image)
        image = self.image_queue.get(block=True, timeout=1)
        cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        frame = cv2_img.copy()
        frame_result = frame

        frame_result = run(frame)
        cv2.imshow('image', frame_result)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    apriltag_node = ApriltagNode()
    rclpy.spin(apriltag_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
        
        
    
