#!/usr/bin/python3
# coding=utf8
# Date:2022/4/28
# Author:hiwonder
import sys
import cv2
import math
import rospy
import threading
import numpy as np
from threading import RLock, Timer
from std_srvs.srv import *
from sensor_msgs.msg import Image
from common import apriltag

ROS_NODE_NAME = 'apriltag_detect_demo'

coordinate = None

tag_id = None
haved_detect = False



detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
camera_intrinsic = np.matrix([  [619.063979, 0,          302.560920],
                                [0,          613.745352, 237.714934],
                                [0,          0,          1]])
# 相机内参(camera internal reference)


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
    cv2.putText(img, tag_id, (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
    return img


def image_callback(ros_image):

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customized image information to image)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame

    frame_result = run(frame)
    cv2.imshow('image', frame_result)
    cv2.waitKey(1)
    

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)


    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher

    rospy.sleep(0.2)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
