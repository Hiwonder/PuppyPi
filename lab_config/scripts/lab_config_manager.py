#!/usr/bin/env python3
import sys
import cv2
import math
import rospy
import json
import yaml
import numpy as np
from threading import RLock, Timer
from sensor_msgs.msg import Image
from std_srvs.srv import *
from std_msgs.msg import Header
from lab_config.srv import *

lock = RLock()
__isRunning = False
image_sub = None
image_pub = None
kernel_erode = 3
kernel_dilate = 3
__color_range_g = {}
current_range = {'min': [0, 0, 0], 'max': [100, 100, 100]}
config_file_path = '/home/ubuntu/puppypi/src/lab_config/config/lab_config.yaml'
heartbeat_timer = None
sub_ed = False

# 找出面积最大的轮廓(find out the contour with the maximal area)
# 参数为要比较的轮廓的列表(the parameter is the list of contour to be compared)
def get_area_max_contour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓(iterate through all contours)
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积(calculate contour area)
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 300, the contour with the largest area is considered valid to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour)

def cv2_image2ros(image):
    image_temp = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    image_temp.height = image.shape[:2][0]
    image_temp.width = image.shape[:2][1]
    image_temp.encoding = 'rgb8'
    image_temp.data = image.tobytes()
    image_temp.header = header
    image_temp.step = image_temp.width*3
        
    return image_temp

def image_callback(ros_image):
    global lock, __isRunning, image_pub, current_color_name, current_range
    """ callback of image_sub """
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customize image information to image)
    image_resize = cv2.resize(image, (400, 300), interpolation=cv2.INTER_NEAREST)
    
    frame_result = cv2.cvtColor(image_resize, cv2.COLOR_RGB2LAB)

    range_ = current_range
    # frame_result = cv2.GaussianBlur(frame_result, (3, 3), 3)
    mask = cv2.inRange(frame_result, tuple(range_['min']), tuple(range_['max']))  # 对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
    eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_erode, kernel_erode)))
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_dilate, kernel_dilate)))
    
    gray_image = cv2.cvtColor(dilated, cv2.COLOR_GRAY2RGB)

    rgb_image = np.vstack((gray_image, image_resize))
    ros_img = cv2_image2ros(rgb_image)
     
    image_pub.publish(ros_img)

def enter_func(msg):
    global lock, image_sub, sub_ed
    
    with lock:
        if not sub_ed:
            sub_ed = True
            image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    return [True, 'enter']

def lab_adjust(msg):
    return [True, 'lab_adjust']

def exit_func(msg):
    global lock, image_sub, __isRunning, sub_ed
    
    with lock:
        __isRunning = False
        try:
            sub_ed = False
            image_sub.unregister()
        except:
            pass

    return [True, 'exit']

def start_running():
    global lock, __isRunning
    
    with lock:
        __isRunning = True

def stop_running():
    global lock, __isRunning
    
    with lock:
        __isRunning = False

def set_running(msg):
    if msg:
        start_running()
    else:
        stop_running()
    
    return [True, 'set_running']

def save_to_disk_srv_cb(msg):
    global lock, __color_range_g
    
    with lock:
        cf = {"color_range_list": __color_range_g.copy()}
        rospy.loginfo(cf)
    s = yaml.dump(cf, default_flow_style=False)
    with open(config_file_path, 'w') as f:
        f.write(s)
    rsp = TriggerResponse()
    rsp.success = True

    return rsp

def get_range_srv_cb(msg):
    global lock, __color_range_g
    
    ranges = rospy.get_param('~color_range_list', __color_range_g)
    rsp = GetRangeResponse()
    if msg.color_name in ranges:
        rsp.success = True
        rsp.min = ranges[msg.color_name]['min']
        rsp.max = ranges[msg.color_name]['max']
    else:
        rsp.success = False
    with lock:
        __color_range_g = ranges
    
    return rsp

def change_range_srv_cb(msg):
    global current_range
    
    with lock:
        current_range = dict(min=list(msg.min), max=list(msg.max))
    rsp = ChangeRangeResponse()
    rsp.success = True
    
    return rsp

def stash_range_srv_cb(msg):
    global lock, __color_range_g, current_range
    
    ranges = rospy.get_param('~color_range_list', __color_range_g)
    with lock:
        ranges[msg.color_name] = current_range.copy()
    rospy.set_param('~color_range_list', ranges)
    __color_range_g = ranges
    rsp = StashRangeResponse()
    rsp.success = True
    
    return rsp

def get_all_color_name_srv_cb(msg):
    global lock, __color_range_g, current_range
    
    ranges = rospy.get_param('~color_range_list', __color_range_g)
    color_names = list(ranges.keys())
    rsp = GetAllColorNameResponse()
    rsp.color_names = color_names
    
    return rsp

def heartbeat_srv_cb(msg):
    global heartbeat_timer
    
    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    rospy.logdebug("Heartbeat")
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/lab_config_manager/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data
    
    return rsp

if __name__ == '__main__':
    rospy.init_node('lab_config_manager', log_level=rospy.INFO)

    __color_range_g = rospy.get_param('~color_range_list', {})
    
    kernel_erode = rospy.get_param('~kernel_erode', 3)
    kernel_dilate = rospy.get_param('~kernel_dilate', 3)
    
    config_file_path = rospy.get_param('~config_file_path', config_file_path)
    
    if 'red' in __color_range_g:
        current_range = __color_range_g['red']

    image_pub = rospy.Publisher('/lab_config_manager/image_result', Image, queue_size=1)
    enter_srv = rospy.Service('/lab_config_manager/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/lab_config_manager/exit', Trigger, exit_func)
    have_lab_adjust = rospy.Service('/lab_config_manager/lab_adjust', Trigger, lab_adjust)
    running_srv = rospy.Service('/lab_config_manager/set_running', SetBool, set_running)

    save_to_disk_srv = rospy.Service('lab_config_manager/save_to_disk', Trigger, save_to_disk_srv_cb)
    get_color_range_srv = rospy.Service('lab_config_manager/get_range', GetRange, get_range_srv_cb)
    change_range_srv = rospy.Service('lab_config_manager/change_range', ChangeRange, change_range_srv_cb)
    stash_range_srv = rospy.Service('lab_config_manager/stash_range', StashRange, stash_range_srv_cb)
    get_all_color_name_srv = rospy.Service('/lab_config_manager/get_all_color_name', GetAllColorName, get_all_color_name_srv_cb)
    heartbeat_srv = rospy.Service('lab_config_manager/heartbeat', SetBool, heartbeat_srv_cb)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
