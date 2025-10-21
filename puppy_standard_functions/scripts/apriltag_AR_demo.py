#!/usr/bin/python3
# coding=utf8
# Date:2021/12/02
# Author:Hiwonder
# 4.ROS+OpenCV视觉识别课程\第8课 AR视觉(4.ROS+OpenCV Visual Recognition Course\Lesson 8 AR Vision)

import os
import cv2
import math
import rospy
import numpy as np
import threading
from common import apriltag
from objloader_simple import OBJ as obj_load
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from scipy.spatial.transform import Rotation as R

ROS_NODE_NAME = 'apriltag_AR_demo'

MODEL_PATH = '/home/ubuntu/puppypi/src/puppy_standard_functions/models'

OBJP = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)

AXIS = np.float32([[-1, -1, 0], 
                   [-1,  1, 0], 
                   [ 1,  1, 0], 
                   [ 1, -1, 0],
                   [-1, -1, 2],
                   [-1,  1, 2],
                   [ 1,  1, 2],
                   [ 1, -1, 2]])


MODELS_SCALE = {
                'bicycle': 50, 
                'fox': 4, 
                'chair': 400, 
                'cow': 0.4,
                'wolf': 0.6,
                }
                # 'pirate-ship-fat': 100}


def draw(img, corners, imgpts):
    imgpts = np.int32(imgpts).reshape(-1,2)
    cv2.drawContours(img, [imgpts[:4]],-1,(0, 255, 0),-3)
    for i,j in zip(range(4),range(4,8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    cv2.drawContours(img, [imgpts[4:]],-1,(0, 0, 255),3)
    return img


class ARNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.model_path = rospy.get_param("~/model_path", MODEL_PATH)
        self.name = name
        self.camera_intrinsic = np.matrix([[619.063979, 0,          302.560920],
                                           [0,          613.745352, 237.714934],
                                           [0,          0,          1]])
        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        self.obj = None
        self.target_model = None
        self.tag_detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())

        
        self.lock = threading.RLock()
        self.image_sub = None
        self.camera_info_sub = None
        self.result_publisher = rospy.Publisher(self.name + '/image_result', Image, queue_size=1)
        

    def enter_srv_callback(self, _):
        rospy.loginfo("enter")
        try:
            if self.image_sub:self.image_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))

        with self.lock:
            self.obj = None
            self.target_model = None
            self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback, queue_size=1)
            
        return TriggerResponse(success=True)

    def exit_srv_callback(self, _):
        rospy.loginfo("exit")
        try:
            if self.image_sub:self.image_sub.unregister()
        except Exception as e:
            rospy.logerr(str(e))
        return TriggerResponse(success=True)
        

    def set_model_srv_callback(self, model):
        with self.lock:
            if model == "":
                self.target_model = None
            else:
                self.target_model = model
                if self.target_model != 'rectangle':
                    obj = obj_load(os.path.join(self.model_path, self.target_model + '.obj'), swapyz=True)
                    obj.faces = obj.faces[::-1]
                    new_faces = []
                    for face in obj.faces:
                        face_vertices = face[0]
                        points = []
                        colors = []
                        for vertex in face_vertices:
                            data = obj.vertices[vertex - 1]
                            points.append(data[:3])
                            if self.target_model != 'cow' and self.target_model != 'wolf':
                                colors.append(data[3:])
                        scale_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * MODELS_SCALE[self.target_model]
                        points = np.dot(np.array(points), scale_matrix)
                        if self.target_model == 'bicycle':
                            points = np.array([[p[0] - 670, p[1] - 350, p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, 180), degrees=True).apply(points)
                        elif self.target_model == 'fox':
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                        elif self.target_model == 'chair':
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                            points = R.from_euler('xyz', (0, 0, -90), degrees=True).apply(points)
                        else:
                            points = np.array([[p[0], p[1], p[2]] for p in points])
                        if len(colors) > 0:
                            color = tuple(255 * np.array(colors[0]))
                        else:
                            color = None
                        new_faces.append((points, color))

                    self.obj = new_faces

    def camera_info_callback(self, msg):
        with self.lock:
            pass

    def image_callback(self, ros_image: Image):
        #rospy.loginfo("image recv")
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, 
                        buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customized image information to image)
        result_image = np.copy(rgb_image)
        
        with self.lock:
            try:
                result_image = self.image_proc(rgb_image, result_image)
            except Exception as e:
                rospy.logerr(str(e))
        

        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)
        
    def image_proc(self, rgb_image, result_image):
        if self.target_model is not None: 
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            detections = self.tag_detector.detect(gray)
            if detections != ():
                
                for detection in detections:
                    M,e0,e1 = self.tag_detector.detection_pose(detection,[self.camera_intrinsic.item(0,0), self.camera_intrinsic.item(1,1),
                                                                self.camera_intrinsic.item(0,2), self.camera_intrinsic.item(1,2)],
                                                                0.033)
                    
                    P = M[:3,:4]
                    coordinate=np.matmul(P,np.array([[0],[0],[0],[1]]))
                    print('coordinate = ',coordinate)

                    tag_id = detection.tag_id
                    tag_center = detection.center
                    tag_corners = detection.corners

                    print('tag_id = ',tag_id)

                    lb = tag_corners[0]
                    rb = tag_corners[1]
                    rt = tag_corners[2]
                    lt = tag_corners[3]
                    cv2.circle(result_image, (int(lb[0]), int(lb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(lt[0]), int(lt[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rb[0]), int(rb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rt[0]), int(rt[1])), 2, (0, 255, 255), -1)
                    
                    corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
                    ret, rvecs, tvecs = cv2.solvePnP(OBJP, corners, self.camera_intrinsic, self.dist_coeffs)
                    if self.target_model == 'rectangle':
                        imgpts, jac = cv2.projectPoints(AXIS, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                        result_image = draw(result_image, corners, imgpts)
                    else:
                         for points, color in self.obj:
                             dst, jac = cv2.projectPoints(points.reshape(-1, 1, 3)/100.0, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                             imgpts = dst.astype(int)
                             # 手动上色(manually apply color)
                             if self.target_model == 'cow':
                                 cv2.fillConvexPoly(result_image, imgpts, (0, 255, 255))
                             elif self.target_model == 'wolf':
                                 cv2.fillConvexPoly(result_image, imgpts, (255, 255, 0))
                             else:
                                 cv2.fillConvexPoly(result_image, imgpts, color)
        return result_image

def main():
    ar_app_node = ARNode('apriltag_AR_demo')
    ar_app_node.enter_srv_callback(1)
    ar_app_node.set_model_srv_callback('bicycle')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main()
