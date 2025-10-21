#!/usr/bin/python3
# coding=utf8
# 第11章 ROS机器狗结合机械臂课程\第3课 颜色识别夹取(11.ROS Robot Combine Robotic Arm Course\Lesson 3 Color Recognition Gripping)
import sys
import cv2
import math
import rospy
import time
import threading
import numpy as np
from threading import RLock, Timer
from ros_robot_controller.msg import BuzzerState
from std_srvs.srv import *
from std_msgs.msg import Float32,Header
from sensor_msgs.msg import Image
from puppy_control.srv import SetRunActionName
from common import Misc


ROS_NODE_NAME = 'color_detect_with_arm'

sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse

color_range_list = {}
detect_color = 'None'
color_list = []

action_finish = True


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'yellow': (0, 255, 255),
    'white': (255, 255, 255),
}
draw_color = range_rgb["yellow"]

# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(the parameter is the list of contour to be compared)
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate the contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 50, the contour with the maximal area is valid to filter the interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

# 初始位置(initial position)
def initMove():
    runActionGroup_srv('look_down.d6a',True)
    
target_color = 'red'
def move():
    global action_finish 
    global draw_color
    global detect_color
    global target_color
    
    while not rospy.is_shutdown():
            
        if detect_color == target_color:
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.1
            msg.off_time = 0.9
            msg.repeat = 1
            buzzer_pub.publish(msg)
            action_finish = False
            rospy.sleep(0.8)
            runActionGroup_srv('grab.d6a', True)
            rospy.sleep(0.5)
            setServoPulse(9,1200,300)
            rospy.sleep(0.3)
            setServoPulse(9,1500,300)
            runActionGroup_srv('look_down.d6a', True)
                                
            rospy.sleep(0.8)
            detect_color = 'None'
            draw_color = range_rgb["yellow"]
        action_finish = True                
        rospy.sleep(0.01)  
        
    runActionGroup_srv('stand_with_arm.d6a', True) 
    
  # 运行子线程(run sub-thread)
th = threading.Thread(target=move,daemon=True)
th.start()      
        

def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish 
    global target_color
    
    size = (320, 240)

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    if action_finish:

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
                dilated[0:120,:] = 0
                dilated[:,0:80] = 0
                dilated[:,240:320] = 0
                # cv2.imshow(i, dilated)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out the contour)
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the contour with the maximal area)
                
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积(find out the maximal area)
                        max_area = area_max
                        #print(max_area)
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 4000:  # 有找到最大面积,面积取大一点，确保色块放入方块里的面积足够大(the largest ares has been found, to ensure that the color block fits well into the square, use a larger area)
            
            ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(get the minimum circumcircle)
            centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
            centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))            
            cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)#画圆(draw circle)
            
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
                    draw_color = range_rgb["yellow"]     
                print('detect_color is',detect_color)          
        else:
            detect_color = 'None'
            draw_color = range_rgb["yellow"]
    cv2.rectangle(img,(190,270),(450,480),(0,255,255),2)
    if detect_color == target_color:
        cv2.putText(img, "Target Color" , (225, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, draw_color, 2)  
    else:
        cv2.putText(img, "Not Target Color" , (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, draw_color, 2)      
    cv2.putText(img, "Color: " + detect_color, (225, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, draw_color, 2)
    
    return img
    

def image_callback(ros_image):

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customized image information to image)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2_img = cv2.flip(cv2_img, 1)
    frame = cv2_img.copy()
    frame_result = run(frame)
    cv2.imshow('Frame', frame_result)
    key = cv2.waitKey(1)
    

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    
    color_range_list = rospy.get_param('/lab_config_manager/color_range_list', {})
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    
    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
    buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)
    initMove()
    rospy.sleep(0.2)
    
    try:
        rospy.spin()
    except :
        rospy.loginfo("Shutting down")
    
