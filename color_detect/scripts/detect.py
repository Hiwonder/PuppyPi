#!/usr/bin/python3
# coding=utf8
# Date:2021/12/02
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
from ros_robot_controller.msg import RGBState, RGBsState
from puppy_control.srv import SetRunActionName

from common import Misc


ROS_NODE_NAME = 'color_detect'
color_range_list = {}
detect_color = 'None'
color_list = []

# 阈值(threshold)
conf_threshold = 0.6

__isRunning = False

org_image_sub_ed = False

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}
draw_color = range_rgb["black"]

action_finish = True

lock = RLock()


# 变量重置(variable reset)
def reset():
    global color_list
    global draw_color
    global detect_color
    global action_finish 
    global color_range_list
    with lock:
        turn_off_rgb()
        detect_color = 'None'
        color_list = []
        draw_color = range_rgb["black"]
        color_range_list = rospy.get_param('/lab_config_manager/color_range_list')


# 初始位置(initial position)
def initMove(delay=True):
    runActionGroup_srv('sit.d6ac',False)
    with lock:
        pass
    if delay:
        rospy.sleep(2)

def turn_off_rgb():
    led1 = RGBState()
    led1.id = 1
    led1.r = 0
    led1.g = 0
    led1.b = 0

    led2 = RGBState()
    led2.id = 2
    led2.r = 0
    led2.g = 0
    led2.b = 0

    msg = RGBsState()
    msg.data = [led1, led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.005)
    

def turn_on_rgb(color):
    led1 = RGBState()
    led1.id = 1
    led1.r = range_rgb[color][2]
    led1.g = range_rgb[color][1]
    led1.b = range_rgb[color][0]

    led2 = RGBState()
    led2.id = 2
    led2.r = range_rgb[color][2]
    led2.g = range_rgb[color][1]
    led2.b = range_rgb[color][0]

    msg = RGBsState()
    msg.data = [led1, led2]
    rgb_pub.publish(msg)
    rospy.sleep(0.005)

# app初始化调用(app initialization calling)
def init():
    print("color detect Init")
    initMove(False)
    reset()



def move():
    global action_finish 
    global draw_color
    global detect_color
    while True:
        if __isRunning:
            if detect_color != 'None':
                action_finish = False
                turn_on_rgb(detect_color)
                if detect_color == 'red':
                    runActionGroup_srv('nod.d6ac', True)
                    # rospy.sleep(1.7)
                    detect_color = 'None'
                    draw_color = range_rgb["black"]                    
                    rospy.sleep(1)
                elif detect_color == 'green' or detect_color == 'blue':
                    runActionGroup_srv('shake_head.d6ac', True)
                    # rospy.sleep(2.1)
                    detect_color = 'None'
                    draw_color = range_rgb["black"]
                    rospy.sleep(1)
                else:
                    rospy.sleep(0.01)                
                action_finish = True                
                detect_color = 'None'
            else:
                rospy.sleep(0.01)         
        else:
            rospy.sleep(0.01)
            
# 运行子线程(run sub-thread)
th = threading.Thread(target=move,daemon=True)
th.start()

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
            if contour_area_temp > 50:  # 只有在面积大于50时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 50, the contour with the largest area is considered valid to filter out interference.)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour)
def run(img):
    global draw_color
    global color_list
    global detect_color
    global action_finish
    size = (320, 240)

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

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
                # if debug:
                #     cv2.imshow(i, dilated)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out contour)
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the maximal contour)
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积(find the max area)
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        if max_area > 200:  # 有找到最大面积(find out the maximal area)
            ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(get the minimal circumcircle)
            centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
            centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))            
            cv2.circle(img, (centerX, centerY), radius, range_rgb[color_area_max], 2)#画圆(draw circle)

            if color_area_max == 'red':  #红色最大(red area is max)
                color = 1
            elif color_area_max == 'green':  #绿色最大(green area is max)
                color = 2
            elif color_area_max == 'blue':  #蓝色最大(blue area is max)
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
        else:
            detect_color = 'None'
            draw_color = range_rgb["black"]
            
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img

def image_callback(ros_image):
    global lock
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert hte customize image information to image)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame
    with lock:
        if __isRunning:
            frame_result = run(frame)
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tobytes()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)

def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("enter color detect")
    with lock:
        init()
        if not org_image_sub_ed:
            org_image_sub_ed = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
            
    return [True, 'enter']

heartbeat_timer = None
def exit_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed
    
    rospy.loginfo("exit color detect")
    with lock:
        __isRunning = False
        reset()
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                if heartbeat_timer:heartbeat_timer.cancel()
                image_sub.unregister()
        except:
            pass
    
    return [True, 'exit']

def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running color detect")
    with lock:
        __isRunning = True

def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running color detect")
    with lock:
        __isRunning = False
        reset()
        initMove(delay=False)

def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()
    
    return [True, 'set_running']

def heartbeat_srv_cb(msg):
    global heartbeat_timer
    
    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/%s/exit'%ROS_NODE_NAME, Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)

    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher
    rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)

    enter_srv = rospy.Service('/%s/enter'%ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit'%ROS_NODE_NAME, Trigger, exit_func)
    running_srv = rospy.Service('/%s/set_running'%ROS_NODE_NAME, SetBool, set_running)
    heartbeat_srv = rospy.Service('/%s/heartbeat'%ROS_NODE_NAME, SetBool, heartbeat_srv_cb)
    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)

    debug = False
    if debug:
        enter_func(1)
        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
