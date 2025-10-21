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
from common import apriltag
from puppy_control.srv import SetRunActionName


ROS_NODE_NAME = 'apriltag_detect'

# 用到的动作组名称(the used name of action group)
id_1_action = 'shake_hands.d6ac'
id_2_action = 'lie_down.d6ac'
id_3_action = 'push-up.d6ac'

tag_id = None
haved_detect = False

__isRunning = False

org_image_sub_ed = False


action_finish = True

lock = RLock()

detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())

# 变量重置(variable reset)
def reset():
    global tag_id
    global haved_detect
    with lock:
        tag_id = None
        haved_detect = False


# 初始位置(initial position)
def initMove(delay=False):
    runActionGroup_srv('sit.d6ac',False)
    # runActionGroup('sit.d6ac')
    with lock:
        pass
    if delay:
        rospy.sleep(2)

# app初始化调用(app initialization calling)
def init():
    print("color detect Init")
    initMove()
    reset()

def move():
    global haved_detect
    while True:
        if __isRunning:
            if haved_detect:
                if tag_id == '1':
                    runActionGroup_srv(id_1_action, True)
                elif tag_id == '2':
                    runActionGroup_srv(id_2_action, True)
                elif tag_id == '3':
                    runActionGroup_srv(id_3_action, True)

                haved_detect = False
            else:
                rospy.sleep(0.01)         
        else:
            rospy.sleep(0.01)
            
# 运行子线程(run sub-thread)
th = threading.Thread(target=move,daemon=True)
th.start()


def apriltagDetect(img):   
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray, return_image=False)
    if len(detections) != 0:
        for detection in detections:                       
            corners = np.rint(detection.corners)  # 获取四个角点(obtain four corners)
            cv2.drawContours(img, [np.array(corners, int)], -1, (0, 255, 255), 5, cv2.LINE_AA)
            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family(get tag_family)

            if tag_family == 'tag36h11':
                tag_id = str(detection.tag_id)  # 获取tag_id(get tag_id)
                return tag_id
            else:
                return None
    else:
        return None

def run(img):
    global tag_id
    global haved_detect

    if not __isRunning:
        return img
    
    tag_id = apriltagDetect(img) # apriltag检测(apriltag detection)
    if tag_id is not None and not haved_detect:
        haved_detect = True
    cv2.putText(img, tag_id, (10, img.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 255), 3)
    return img


def image_callback(ros_image):
    global lock
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customize image information to image)
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

    rospy.loginfo("enter apriltag detect")
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
    
    rospy.loginfo("exit apriltag detect")
    with lock:
        __isRunning = False
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                heartbeat_timer.cancel()
                image_sub.unregister()
        except:
            pass
    
    return [True, 'exit']

def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running apriltag detect")
    with lock:
        __isRunning = True

def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running apriltag detect")
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
