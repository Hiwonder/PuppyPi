#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/10/22
import cv2
import math
import yaml
import numpy as np
import transforms3d as tfs
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
import random

range_rgb = {
    'red': (0, 50, 255),
    'blue': (255, 50, 0),
    'green': (50, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

def loginfo(msg):
    Node.get_logger().info('\033[1;32m%s\033[0m' % msg)

def val_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def empty_func(img=None):
    return img

def set_range(x, x_min, x_max):
    tmp = x if x > x_min else x_min
    tmp = tmp if tmp < x_max else x_max
    return tmp

def get_yaml_data(yaml_file):
    yaml_file = open(yaml_file, 'r', encoding='utf-8')
    file_data = yaml_file.read()
    yaml_file.close()

    data = yaml.load(file_data, Loader=yaml.FullLoader)

    return data


def save_yaml_data(data, yaml_file):
    f = open(yaml_file, 'w', encoding='utf-8')
    yaml.dump(data, f)

    f.close()


def distance(point_1, point_2):
    """
    计算两个点间的距离(calculate the distance between two points)
    :param point_1: 点1
    :param point_2: 点2
    :return: 两点间的距离(distance between two points)
    """
    return math.sqrt((point_1[0] - point_2[0]) ** 2 + (point_1[1] - point_2[1]) ** 2)


def box_center(box):
    """
    计算四边形box的中心(calculate the center of quadrangle box)
    :param box: box （x1, y1, x2, y2)形式(box （x1, y1, x2, y2)type)
    :return:  中心坐标（x, y)(center coordinate（x, y))
    """
    return (box[0] + box[2]) / 2, (box[1] + box[3]) / 2


def bgr8_to_jpeg(value, quality=75):
    """
    将cv bgr8格式数据转换为jpg格式(convert data in the format of cv bgr8 into jpg)
    :param value: 原始数据(original data)
    :param quality:  jpg质量 最大值100(jpg quality. Maximum value is 100)
    :return:
    """
    return bytes(cv2.imencode('.jpg', value)[1])


def point_remapped(point, now, new, data_type=float):
    """
    将一个点的坐标从一个图片尺寸映射的新的图片上(map the coordinate of one point from a picture to a new picture of different size)
    :param point: 点的坐标(coordinate of point)
    :param now: 现在图片的尺寸(size of current picture)
    :param new: 新的图片尺寸(new picture size)
    :return: 新的点坐标(new point coordinate)
    """
    x, y = point
    now_w, now_h = now
    new_w, new_h = new
    new_x = x * new_w / now_w
    new_y = y * new_h / now_h
    return data_type(new_x), data_type(new_y)


def get_area_max_contour(contours, threshold=50):
    """
    获取轮廓中面积最重大的一个, 过滤掉面积过小的情况(get the contour whose area is the largest. Filter out those whose area is too small)
    :param contours: 轮廓列表(contour list)
    :param threshold: 面积阈值, 小于这个面积的轮廓会被过滤(area threshold. Contour whose area is less than this value will be filtered out)
    :return: 如果最大的轮廓面积大于阈值则返回最大的轮廓, 否则返回None(if the maximum contour area is greater than this threshold, return the
    largest contour, otherwise return None)
    """
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > threshold:  # 过滤干扰
                area_max_contour = c

    return area_max_contour,  contour_area_max  # 返回最大的轮廓


def vector_2d_angle(v1, v2):
    """
    计算两向量间的夹角 -pi ~ pi(calculate the angle between two vectors -pi ~ pi)
    :param v1: 第一个向量(first vector)
    :param v2: 第二个向量(second vector)
    :return: 角度(angle)
    """
    d_v1_v2 = np.linalg.norm(v1) * np.linalg.norm(v2)
    cos = v1.dot(v2) / (d_v1_v2)
    sin = np.cross(v1, v2) / (d_v1_v2)
    angle = np.degrees(np.arctan2(sin, cos))
    return angle


def warp_affine(image, points, scale=1.0):
    """
     简单的对齐，计算两个点的连线的角度，以图片中心为原点旋转图片，使线水平(Simple alignment. Calculate the angle of the line connecting the two points.
     Rotate the picture around the image center to make the line horizontal)
    可以用在人脸对齐上(can be used to align the face)

    :param image: 要选择的人脸图片(select face picture)
    :param points: 两个点的坐标 ((x1, y1), (x2, y2))(coordinate of two points ((x1, y1), (x2, y2)))
    :param scale: 缩放比例(scaling)
    :return: 旋转后的图片(rotated picture)
    """
    w, h = image.shape[:2]
    dy = points[1][1] - points[0][1]
    dx = points[1][0] - points[0][0]
    # 计算旋转角度并旋转图片(calculate the rotation angle and rotate picture)
    angle = cv2.fastAtan2(dy, dx)
    rot = cv2.getRotationMatrix2D((int(w / 2), int(h / 2)), angle, scale=scale)
    return cv2.warpAffine(image, rot, dsize=(h, w))


class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))

colors = Colors()  # create instance for 'from utils.plots import colors'

def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param:
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    if x != None:
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )

def qua2rpy(qua):
    if type(qua) == Quaternion:
        x, y, z, w = qua.x, qua.y, qua.z, qua.w
    else:
        x, y, z, w = qua[0], qua[1], qua[2], qua[3]
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
  
    return roll, pitch, yaw

def rpy2qua(roll, pitch, yaw):
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    
    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation

def xyz_quat_to_mat(xyz, quat):
    mat = tfs.quaternions.quat2mat(np.asarray(quat))
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def xyz_rot_to_mat(xyz, rot):
    return np.row_stack((np.column_stack((rot, xyz)), np.array([[0, 0, 0, 1]])))

def xyz_euler_to_mat(xyz, euler, degrees=True):
    if degrees:
        mat = tfs.euler.euler2mat(math.radians(euler[0]), math.radians(euler[1]), math.radians(euler[2]))
    else:
        mat = tfs.euler.euler2mat(euler[0], euler[1], euler[2])
    mat = tfs.affines.compose(np.squeeze(np.asarray(xyz)), mat, [1, 1, 1])
    return mat

def mat_to_xyz_euler(mat, degrees=True):
    t, r, _, _ = tfs.affines.decompose(mat)
    if degrees:
        euler = np.degrees(tfs.euler.mat2euler(r))
    else:
        euler = tfs.euler.mat2euler(r)
    return t, euler
