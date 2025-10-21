#!/usr/bin/env python3
# encoding: utf-8
# @data:2024/090/13
# @author:liuyuan
# 颜色检测
import cv2
import math
import yaml
import queue
import rclpy
import threading
import numpy as np
import sdk.common as common
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter

bridge = CvBridge()
# 颜色检测

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

lab_data = common.get_yaml_data("/home/ubuntu/software/lab_tool/lab_config.yaml")

color_list = []
detect_color = 'None'
draw_color = range_rgb["black"]



size = (320, 240)
def run(img):
    global draw_color
    global color_list
    global detect_color
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    for i in ['red', 'green', 'blue']:
        #frame_mask = cv2.inRange(frame_lab, tuple(lab_data['lab']['Stereo'][i]['min']), tuple(lab_data['lab']['Stereo'][i]['max']))  #对原图像和掩模进行位运算
        frame_mask = cv2.inRange(frame_lab, tuple(lab_data['color_range_list'][i]['min']), tuple(lab_data['color_range_list'][i]['max'])) 
        eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓
        areaMaxContour, area_max = common.get_area_max_contour(contours, 0)  #找出最大轮廓
        if areaMaxContour is not None:
            if area_max > max_area:#找最大面积
                max_area = area_max
                color_area_max = i
                areaMaxContour_max = areaMaxContour
    if max_area > 200:  # 有找到最大面积
        ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆
        centerX = int(common.val_map(centerX, 0, size[0], 0, img_w))
        centerY = int(common.val_map(centerY, 0, size[1], 0, img_h))
        radius = int(common.val_map(radius, 0, size[0], 0, img_w))            
        cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)#画圆

        if color_area_max == 'red':  #红色最大
            color = 1
        elif color_area_max == 'green':  #绿色最大
            color = 2
        elif color_area_max == 'blue':  #蓝色最大
            color = 3
        else:
            color = 0
        color_list.append(color)

        if len(color_list) == 3:  #多次判断
            # 取平均值
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
    else:
        detect_color = 'None'
        draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img

image_queue = queue.Queue(2)
def image_callback(ros_image):
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
    bgr_image = np.array(cv_image, dtype=np.uint8)
    if image_queue.full():
        # 如果队列已满，丢弃最旧的图像
        image_queue.get()
        # 将图像放入队列
    image_queue.put(bgr_image)

def main():
    running = True
    while running:
        try:
            image = image_queue.get(block=True, timeout=1)
        except queue.Empty:
            if not running:
                break
            else:
                continue
        image = run(image)
        cv2.imshow('image', image)
        key = cv2.waitKey(1)
        if key == ord('q') or key == 27:  # 按q或者esc退出
            break

    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    rclpy.init()
    node = rclpy.create_node('color_detect_node')
    node.create_subscription(Image, 'image_raw', image_callback, 1)
    threading.Thread(target=main, daemon=True).start()
    rclpy.spin(node)
