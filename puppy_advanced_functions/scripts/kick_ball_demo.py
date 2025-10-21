#!/usr/bin/python3
# coding=utf8
# 第7章 ROS机器狗创意课程\3.AI自主追踪踢球(7.ROS Robot Creative Lesson\3.AI Auto Shooting)
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
from ros_robot_controller.msg import RGBState, RGBsState
from object_tracking.srv import *
from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName

ROS_NODE_NAME = 'kick_ball_demo'
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
    LOOKING_FOR = 0 #寻找 先低头寻找，没有就跳转到LOOKING_FOR_LEFT向左一步寻找，再没有的话就跳转到LOOKING_FOR_RIGHT，一直向右寻找(search, start by looking down,  If nothing is found, transition to LOOKING_FOR_LEFT to search one step to the left. If still nothing is found, transition to LOOKING_FOR_RIGHT and continue searching to the right)
    LOOKING_FOR_LEFT = 1
    LOOKING_FOR_RIGHT = 2 
    FOUND_TARGET = 3 # 已经发现目标(target acquired)
    CLOSE_TO_TARGET = 4 # 靠近目标(approach target)
    CLOSE_TO_TARGET_FINE_TUNE = 5 # 细调(fine-tuning)
    KICK_BALL = 6 # 踢球(shooting)
    STOP = 10
    END = 20            

puppyStatus = PuppyStatus.LOOKING_FOR
puppyStatusLast = PuppyStatus.END


expect_center = {'X':640/2,'Y':480/2} #
expect_center_kick_ball_left = {'X':150,'Y':480-150} # 左脚踢球，希望小球到达此坐标开始踢(left foot kicking the ball, aiming for it to reach this coordinate before starting)
expect_center_kick_ball_right = {'X':640-150,'Y':480-150}# 右脚踢球，希望小球到达此坐标开始踢(right foot kicking the ball, aiming for it to reach this coordinate before starting)
target_info = None # 小球中心点坐标(coordinates of the center point of the ball)

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
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 300, the contour with the largest area is considered valid to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour)


def move():
    global detect_color
    global puppyStatus, puppyStatusLast, haved_detect, action_finish, target_info, PuppyPose
    time.sleep(2)

    while True:
        time.sleep(0.01)
        while(puppyStatus == PuppyStatus.LOOKING_FOR) :
            if haved_detect:
                puppyStatus = PuppyStatus.FOUND_TARGET
                break
            with lock:
                PuppyPose = PP['LookDown_10deg'].copy()
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
                time.sleep(0.2)
            time.sleep(0.8)
            puppyStatus = PuppyStatus.LOOKING_FOR_LEFT
            break
        while(puppyStatus == PuppyStatus.LOOKING_FOR_LEFT) :
            if haved_detect:
                puppyStatus = PuppyStatus.FOUND_TARGET
                break
            with lock:
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(10))
                time.sleep(3)
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))
                time.sleep(0.3)
            time.sleep(0.8)
            puppyStatus = PuppyStatus.LOOKING_FOR_RIGHT
            break
        while(puppyStatus == PuppyStatus.LOOKING_FOR_RIGHT) :
            if haved_detect:
                puppyStatus = PuppyStatus.FOUND_TARGET
                break

            PuppyPose = PP['LookDown_10deg'].copy()
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
            time.sleep(0.2)
            PuppyVelocityPub.publish(x=2, y=0, yaw_rate = math.radians(-12))
            break
        while(puppyStatus == PuppyStatus.FOUND_TARGET) :
            # with lock:
            if target_info['centerY'] > 380:
                puppyStatus = PuppyStatus.CLOSE_TO_TARGET
                PuppyPose = PP['LookDown_20deg'].copy()
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
                time.sleep(0.2)
                break
            if expect_center['X'] - target_info['centerX'] < -80:
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(-12))
                time.sleep(0.2)
            elif expect_center['X'] - target_info['centerX'] > 80:
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(12))
                time.sleep(0.2)
            else:
                PuppyVelocityPub.publish(x=10, y=0, yaw_rate = math.radians(0))
                time.sleep(0.2)
            break
        while(puppyStatus == PuppyStatus.CLOSE_TO_TARGET) :
            # with lock:
            if target_info['centerY'] > 380:
                PuppyMove['x'] = 0
                PuppyMove['yaw_rate'] = math.radians(0)
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))
                puppyStatus = PuppyStatus.CLOSE_TO_TARGET_FINE_TUNE
                print('expect_center[X] , target_info[centerX]',expect_center['X'] , target_info['centerX'])
                if expect_center['X'] > target_info['centerX']:
                    which_foot_kick_ball = 'left'
                else:
                    which_foot_kick_ball = 'right'
                print(which_foot_kick_ball)
                break
            if expect_center['X'] - target_info['centerX'] < -50:
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(-10))
                time.sleep(0.2)
            elif expect_center['X'] - target_info['centerX'] > 50:
                PuppyVelocityPub.publish(x=3, y=0, yaw_rate = math.radians(10))
                time.sleep(0.2)
            else:
                PuppyVelocityPub.publish(x=8, y=0, yaw_rate = math.radians(0))
                time.sleep(0.2)
            # print(target_info)
            break
        
        while(puppyStatus == PuppyStatus.CLOSE_TO_TARGET_FINE_TUNE) :

            if target_info['centerY'] < expect_center_kick_ball_left['Y']:
                PuppyVelocityPub.publish(x=4, y=0, yaw_rate = math.radians(0))
                time.sleep(0.1)
            elif which_foot_kick_ball == 'left' and target_info['centerX'] > expect_center_kick_ball_left['X']:
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(-8))
                time.sleep(0.1)
            elif which_foot_kick_ball == 'right' and target_info['centerX'] < expect_center_kick_ball_right['X']:
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(8))
                time.sleep(0.1)
            else:# 最后一次微调(the final fine-tuning)
                if which_foot_kick_ball == 'left':
                    PuppyVelocityPub.publish(x=5, y=0, yaw_rate = math.radians(-10))
                else:
                    PuppyVelocityPub.publish(x=5, y=0, yaw_rate = math.radians(10))
                time.sleep(1.8)
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))
                puppyStatus = PuppyStatus.KICK_BALL
            # PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))
            # time.sleep(0.1)#停下来需要稳定的时间(the time required to come to a stable stop)
            time.sleep(0.1)
            break
        while(puppyStatus == PuppyStatus.KICK_BALL) :
            with lock:
                PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))
                time.sleep(0.2)
                if which_foot_kick_ball == 'left':
                    runActionGroup_srv('kick_ball_left.d6ac',True)
                else:
                    runActionGroup_srv('kick_ball_right.d6ac',True)
                puppyStatus = PuppyStatus.LOOKING_FOR
                haved_detect = False

        if puppyStatus == PuppyStatus.STOP:
            PuppyMove['x'] = 0
            PuppyMove['yaw_rate'] = math.radians(0)
            PuppyVelocityPub.publish(x=0, y=0, yaw_rate = math.radians(0))

        
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
    global draw_color
    global color_list
    global detect_color
    global action_finish
    global haved_detect
    global target_info 
    # img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)      
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)

    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0
    
    if True:#action_finish
        for i in color_range_list:
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab,
                                             (color_range_list[i]['min'][0],
                                              color_range_list[i]['min'][1],
                                              color_range_list[i]['min'][2]),
                                             (color_range_list[i]['max'][0],
                                              color_range_list[i]['max'][1],
                                              color_range_list[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
                if debug:
                    cv2.imshow(i, dilated)
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out the contour)
                areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the maximal contour)
                if areaMaxContour is not None:
                    if area_max > max_area:#找最大面积(find the maximal area)
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
                       
        if max_area > 200:  # 200  

            rect = cv2.minAreaRect(areaMaxContour_max)#最小外接矩形(the minimal bounding rectangle)
            
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
            centerX = int(Misc.map(rect[0][0], 0, size[0], 0, img_w))
            centerY = int(Misc.map(rect[0][1], 0, size[1], 0, img_h))
            sideX = int(Misc.map(rect[1][0], 0, size[0], 0, img_w))
            sideY = int(Misc.map(rect[1][1], 0, size[1], 0, img_h))
            angle = rect[2]
            for i in range(4):
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):                
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形(draw a rectangle formed by connecting the four points)


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
                # 取平均值(take the average value)
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
        if detect_color == 'red':
            haved_detect = True
            if sideX > sideY:
                target_info = {'centerX':centerX, 'centerY':centerY, 'sideX':sideX, 'sideY':sideY, 'scale':sideX/sideY, 'angle':angle}
            else:
                target_info = {'centerX':centerX, 'centerY':centerY, 'sideX':sideX, 'sideY':sideY, 'scale':sideY/sideX, 'angle':angle}
        else:
            haved_detect = False
    cv2.putText(img, "Color: " + detect_color, (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2)
    
    return img


def image_callback(ros_image):
    # global lock
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customize image information to image)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame
    with lock:
        if __isRunning:
            frame_result = run(frame)
            cv2.imshow('Frame', frame_result)
            key = cv2.waitKey(1)

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
    
    GaitConfig = {'overlap_time':0.15, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':3}

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher


    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)

    rospy.sleep(0.3)
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

