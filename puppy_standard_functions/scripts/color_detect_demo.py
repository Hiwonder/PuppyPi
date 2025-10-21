#!/usr/bin/python3
# coding=utf8
# Date:2022/4/28
# Author:hiwonder
# 第6章 ROS机器狗标准课程\4.ROS+OpenCV视觉识别课程\第6课 颜色识别(6.ROS Robot Standard Course\4.ROS+OpenCV Visual Recognition Course\Lesson 6 Color Recognition)
import sys
import cv2
import math
import rospy
import threading
import numpy as np
from threading import RLock, Timer
from std_srvs.srv import *
from sensor_msgs.msg import Image
from common import Misc


ROS_NODE_NAME = 'color_detect_demo'

color_range_list = {}
detect_color = 'None'
color_list = []

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
draw_color = range_rgb["black"]

# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(the parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 50, the contour with the maximal area is considered valid to filter the interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)
def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish
    size = (320, 240)

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    for i in color_range_list:
        if i in ['red', 'green', 'blue']:
            frame_mask = cv2.inRange(frame_lab,
                                            (color_range_list[i]['min'][0],
                                            color_range_list[i]['min'][1],
                                            color_range_list[i]['min'][2]),
                                            (color_range_list[i]['max'][0],
                                            color_range_list[i]['max'][1],
                                            color_range_list[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
            # if debug:
            #     cv2.imshow(i, dilated)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out contour)
            areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the contour with the maximal area)
            if areaMaxContour is not None:
                if area_max > max_area:#找最大面积(find the maximal area)
                    max_area = area_max
                    color_area_max = i
                    areaMaxContour_max = areaMaxContour
    if max_area > 200:  # 有找到最大面积(the maximal area is found)
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(get the minimum circumcircle)
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        radius = int(Misc.map(radius, 0, size[0], 0, img_w))            
        cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)#画圆(draw circle)
        print("target_color_x: " + str(centerX) + " target_color_y: " + str(centerY) + " target_color_radius: " + str(radius))

        if color_area_max == 'red':  #红色最大(red is the maximal area)
            color = 1
        elif color_area_max == 'green':  #绿色最大(green is the maximal area)
            color = 2
        elif color_area_max == 'blue':  #蓝色最大(blue is the maximal area)
            color = 3
        else:
            color = 0
        color_list.append(color)

        if len(color_list) == 3:  #多次判断(multiple judgement)
            # 取平均值(take average value)
            color = int(round(np.mean(np.array(color_list))))
            color_list = []
            if color == 1:
                detect_color = 'red'
                draw_color = range_rgb["red"]
            elif color == 2:
                detect_color = 'green'
                draw_color = range_rgb["green"]
            elif color == 3:
                detect_color = 'blue'
                draw_color = range_rgb["blue"]
            else:
                detect_color = 'None'
                draw_color = range_rgb["black"]     
            print('detect_color is',detect_color)          
    else:
        detect_color = 'None'
        draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img

def image_callback(ros_image):

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customized image information to image)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame

    frame_result = run(frame)
    cv2.imshow('Frame', frame_result)
    key = cv2.waitKey(1)
    # rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tobytes()
    # ros_image.data = rgb_image
    # image_pub.publish(ros_image)



if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)

    color_range_list = rospy.get_param('/lab_config_manager/color_range_list', {})
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
