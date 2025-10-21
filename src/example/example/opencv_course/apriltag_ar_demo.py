#!/usr/bin/python3
# coding=utf8
# Date:2024/09/03
# Author:liuyuan
# AR视觉
import os
import cv2
import math
import numpy as np
import threading
from sdk import apriltag
import rclpy
from rclpy.node import Node
from sdk.objloader_simple import OBJ as obj_load
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
print(apriltag.__file__)

bridge = CvBridge()
ROS_NODE_NAME = 'apriltag_AR_demo'
MODEL_PATH = '/home/ubuntu/ros2_ws/src/example/example/opencv_course/models'

# 对象内部控制点坐标
objp = np.array([[-1, -1,  0],
                 [ 1, -1,  0],
                 [-1,  1,  0],
                 [ 1,  1,  0],
                 [ 0,  0,  0]], dtype=np.float32)
# 世界坐标系参考点
axis = np.float32([[-1, -1, 0], 
                   [-1,  1, 0], 
                   [ 1,  1, 0], 
                   [ 1, -1, 0],
                   [-1, -1, 2],
                   [-1,  1, 2],
                   [ 1,  1, 2],
                   [ 1, -1, 2]])
# 不同物体模型在真实世界的缩放尺度                   
models_scale = {
                'bicycle': 50, 
                'fox': 4, 
                'chair': 400, 
                'cow': 0.4,
                'wolf': 0.6,
                }


def draw(img, corners, imgpts):

    imgpts = np.int32(imgpts).reshape(-1,2)
    cv2.drawContours(img, [imgpts[:4]],-1,(0, 255, 0),-3)
    for i,j in zip(range(4),range(4,8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]),(255),3)
    cv2.drawContours(img, [imgpts[4:]],-1,(0, 0, 255),3)
    return img
    

class ARNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.model_path = self.declare_parameter('model_path', MODEL_PATH).value
        self.name = name
        self.camera_intrinsic = np.matrix([[619.063979, 0, 302.560920],
                                           [0, 613.745352, 237.714934],
                                           [0, 0, 1]])
        self.dist_coeffs = np.array([0.103085, -0.175586, -0.001190, -0.007046, 0.000000])
        self.obj = None
        self.target_model = None
        self.tag_detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())

        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.image_sub = None
        self.camera_info_sub = None

    def enter_srv_callback(self,_):
        self.get_logger().info("enter")
        try:
            if self.image_sub:
                self.image_sub.destroy()
        except Exception as e:
            self.get_logger().error(str(e))

        with self.lock:
            self.obj = None
            self.target_model = None
            self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)


    def exit_srv_callback(self, request, response):
        self.get_logger().info("exit")
        try:
            if self.image_sub:
                self.image_sub.destroy()
        except Exception as e:
            self.get_logger().error(str(e))

        response.success = True
        return response

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
                        scale_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) * models_scale[self.target_model]
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

    def image_callback(self, ros_image):
        #self.get_logger().info("image recv")
        rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='rgb8')
        result_image = np.copy(rgb_image)

        with self.lock:
            try:
                result_image = self.image_proc(rgb_image, result_image)
            except Exception as e:
                self.get_logger().error(str(e))

        cv2.imshow('image', cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

    def image_proc(self, rgb_image, result_image):
        if self.target_model is not None:
            gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
            detections = self.tag_detector.detect(gray)
            if detections != ():
                for detection in detections:
                    # 计算姿态
                    M, e0, e1 = self.tag_detector.detection_pose(detection, [self.camera_intrinsic.item(0, 0), self.camera_intrinsic.item(1, 1),
                                                                              self.camera_intrinsic.item(0, 2), self.camera_intrinsic.item(1, 2)],
                                                                 0.033)
                    # 像素坐标转换
                    P = M[:3, :4]
                    coordinate = np.matmul(P, np.array([[0], [0], [0], [1]]))
                    print('coordinate = ', coordinate)
                    
                    # 获取标记ID和几何信息
                    tag_id = detection.tag_id
                    tag_center = detection.center
                    tag_corners = detection.corners

                    print('tag_id = ', tag_id)

                    lb = tag_corners[0]
                    rb = tag_corners[1]
                    rt = tag_corners[2]
                    lt = tag_corners[3]
                    cv2.circle(result_image, (int(lb[0]), int(lb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(lt[0]), int(lt[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rb[0]), int(rb[1])), 2, (0, 255, 255), -1)
                    cv2.circle(result_image, (int(rt[0]), int(rt[1])), 2, (0, 255, 255), -1)

                    corners = np.array([lb, rb, lt, rt, tag_center]).reshape(5, -1)
                    ret, rvecs, tvecs = cv2.solvePnP(objp, corners, self.camera_intrinsic, self.dist_coeffs)
                    if self.target_model == 'rectangle':
                        imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                        result_image = draw(result_image, corners, imgpts)
                    else:
                        for points, color in self.obj:
                            dst, jac = cv2.projectPoints(points.reshape(-1, 1, 3) / 100.0, rvecs, tvecs, self.camera_intrinsic, self.dist_coeffs)
                            imgpts = dst.astype(int)
                            # 手动上色(manually apply color)
                            if self.target_model == 'cow':
                                cv2.fillConvexPoly(result_image, imgpts, (0, 255, 255))
                            elif self.target_model == 'wolf':
                                cv2.fillConvexPoly(result_image, imgpts, (255, 255, 0))
                            else:
                                cv2.fillConvexPoly(result_image, imgpts, color)
        return result_image

def main(args=None):
    rclpy.init(args=args)
    ar_app_node = ARNode(ROS_NODE_NAME)
    ar_app_node.enter_srv_callback(1)
    ar_app_node.set_model_srv_callback('bicycle')

    try:
        rclpy.spin(ar_app_node)
    except Exception as e:
        ar_app_node.get_logger().error(str(e))
        ar_app_node.get_logger().info("Shutting down")
    finally:
        ar_app_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
    
