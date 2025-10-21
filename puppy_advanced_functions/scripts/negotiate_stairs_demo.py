#!/usr/bin/python3
# coding=utf8
# 第7章 ROS机器狗创意课程\2.AI识别台阶攀爬(7.ROS Robot Creative Lesson\2.AI Recognition Stair Climbing)

import sys
import cv2
import time
import math
import threading
import numpy as np
from enum import Enum

from common import Misc

import rospy
from std_srvs.srv import *
from sensor_msgs.msg import Image
from object_tracking.srv import *
from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName

ROS_NODE_NAME = 'negotiate_stairs_demo'

is_shutdown = False

color_range_list = rospy.get_param('/lab_config_manager/color_range_list')

PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

lock = threading.Lock()
debug = False
__isRunning = True
haved_detect = False

class PuppyStatus(Enum):
    LOOKING_FOR = 0 #寻找 低头寻找台阶，(search, lowering the head to look for stairs)

    FOUND_TARGET = 3 # 已经发现台阶目标(the target stair has been found)
    DOWN_STAIRS = 4 # 下台阶(go down the stair)

    STOP = 10
    END = 20            

puppyStatus = PuppyStatus.LOOKING_FOR
puppyStatusLast = PuppyStatus.END



target_centre_point = None # 目标中心点坐标(the coordinates of the target center point)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

color_list = []
detect_color = 'None'
action_finish = True
draw_color = range_rgb["black"]
__target_color = ('red',)


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
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 50, the contour with the largest area is considered valid to filter out interference)
                area_max_contour = c
    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour)


def move():
    global detect_color
    global puppyStatus, puppyStatusLast, haved_detect, action_finish, target_centre_point, PuppyPose
    up_stairs_time = 0
    rospy.sleep(2)
    while True:
        rospy.sleep(0.01)

        while(puppyStatus == PuppyStatus.LOOKING_FOR) :
            if target_centre_point != None and target_centre_point[1] > 400:
                puppyStatus = PuppyStatus.FOUND_TARGET
                rospy.sleep(2.1) # 继续往前走一点(continue walking forward)
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))
                up_stairs_time = time.time()
                break
            
            PuppyVelocityPub.publish(x=10, y=0, yaw_rate = math.radians(0))
            
            rospy.sleep(0.01)
            break
        
        while(puppyStatus == PuppyStatus.FOUND_TARGET) :
            runActionGroup_srv('up_stairs_2cm.d6ac',True)
            if time.time() - up_stairs_time > 25:
                puppyStatus = PuppyStatus.DOWN_STAIRS
                PuppyPose = PP['Stand'].copy()
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
                rospy.sleep(0.5)
            break

        while(puppyStatus == PuppyStatus.DOWN_STAIRS) :
            PuppyVelocityPub.publish(x=14, y=0, yaw_rate = math.radians(0))
            rospy.sleep(0.1)
            break

        if puppyStatusLast != puppyStatus:
            print('puppyStatus',puppyStatus)
        puppyStatusLast = puppyStatus

        if is_shutdown:break

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.setDaemon(True)
# th.start()


size = (320, 240)

def run(img):
    global line_centerx
    global __target_color, __isRunning, puppyStatus, target_centre_point, area_max
    
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
            

    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

    for i in color_range_list:
        if i in __target_color:
            detect_color = i
            
            frame_mask = cv2.inRange(frame_lab,
                                            (color_range_list[detect_color]['min'][0],
                                            color_range_list[detect_color]['min'][1],
                                            color_range_list[detect_color]['min'][2]),
                                            (color_range_list[detect_color]['max'][0],
                                            color_range_list[detect_color]['max'][1],
                                            color_range_list[detect_color]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算(opening operation)
            closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算(closing operation)
         
    cnts = cv2.findContours(closed , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓(find out all the contours)
    cnt_large, area_max = getAreaMaxContour(cnts)#找到最大面积的轮廓(find out the contour with the maximal area)
    
    if cnt_large is not None:#如果轮廓不为空(if contour is not none)
        rect = cv2.minAreaRect(cnt_large)#最小外接矩形(the minimum bounding rectangle)
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
        
        centerX = rect[0][0]
        centerY = rect[0][1]
        centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
        centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
        for i in range(4):
            box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
        for i in range(4):                
            box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
            
        cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形(draw a rectangle formed by the four points)
        target_centre_point = [centerX, centerY]      
        cv2.circle(img, (int(centerX), int(centerY)), 5, (0,0,255), -1)#画出中心点(draw the center point)

            
    return img


def image_callback(ros_image):
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customize image information to image)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame
   
    if __isRunning:
        frame_result = run(frame)
        if puppyStatus != PuppyStatus.FOUND_TARGET:
            cv2.imshow('Frame', frame_result)
            key = cv2.waitKey(1)
        rospy.sleep(0.001)

def cleanup():
    global is_shutdown
    is_shutdown = True
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('is_shutdown')

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    rospy.on_shutdown(cleanup)

    PP = rospy.get_param('/puppy_control/PuppyPose')
    PuppyPose = PP['LookDown_10deg'].copy()
    PG = rospy.get_param('/puppy_control/GaitConfig')
    GaitConfig = PG['GaitConfigFast'].copy()

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)

    rospy.sleep(0.5)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    rospy.sleep(0.2)
    
    debug = False
    if debug == False:
        th.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()

