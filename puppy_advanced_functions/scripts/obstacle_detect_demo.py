#!/usr/bin/python3
# coding=utf8
# Date:2022/4/7
# Author:hiwonder
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

ROS_NODE_NAME = 'obstacle_detect_demo'

PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


__isRunning = True
__target_color = ('red',)

trend = 'left' # 遇到障碍物后，判断后续巡线方向的趋势，如果趋势向左，那么小狗向左绕开。如果趋势向右，那么小狗向右绕开(after encountering an obstacle, assess the trend of the following line-following direction. If the trend is towards the left, the robot dog navigates to the left to avoid it. If the trend is towards the right, the robot dog navigates to the right to avoid it)
area_max = 0

class PuppyStatus(Enum):
    NORMAL = 0 # 正常前进中(continuing forward)
    FOUND_TARGET = 3 # 已经发现目标障碍物(target obstacle detected)
    STOP = 10
    END = 20            

puppyStatus = PuppyStatus.NORMAL
puppyStatusLast = PuppyStatus.END



color_range_list = rospy.get_param('/lab_config_manager/color_range_list')

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
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 50, the contour with the largest area is considered valid to filter out interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the maximal contour)


line_centerx = -1
img_centerx = 320
def move():
    global line_centerx, puppyStatus, puppyStatusLast
    
    while True:
        if __isRunning:
            while(puppyStatus == PuppyStatus.NORMAL) :
                if area_max > 70000: # 路桩或障碍物已经在眼前了，已经发现目标，要开始启动避开程序(the road post or obstacle is now in sight, and the target has been detected. It's time to initiate the avoidance program)
                    puppyStatus = PuppyStatus.FOUND_TARGET
                    break
                if line_centerx != -1:
                    if abs(line_centerx - img_centerx) <= 50:
                        PuppyMove['x'] = 10
                        PuppyMove['yaw_rate'] = math.radians(0)
                    elif line_centerx - img_centerx > 50:
                        PuppyMove['x'] = 8
                        PuppyMove['yaw_rate'] = math.radians(-15)
                    elif line_centerx - img_centerx < -50:
                        PuppyMove['x'] = 8
                        PuppyMove['yaw_rate'] = math.radians(15)
                    PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
                break
            while(puppyStatus == PuppyStatus.FOUND_TARGET) :
                signal = 1 if trend == 'left' else -1
                
                PuppyMove['x'] = 0
                PuppyMove['yaw_rate'] = math.radians(15 * signal)
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
                # 开始原地转向，绕开目标物体(begin turning in place to navigate around the target object)
                print('开始原地转向1')
                time.sleep(2)
                area_max_times = 0
                t = time.time()
                while time.time() - t < 5:
                    if area_max < 5000:
                        area_max_times += 1
                        print('area_max',area_max)
                    if area_max_times >= 3:
                        break # 目标物体已经基本离开视野(the target object is almost out of sight)
                    time.sleep(0.1)
                time.sleep(1)
                print('完成原地转向，绕开目标物体',time.time() - t)


                PuppyMove['x'] = 10
                PuppyMove['yaw_rate'] = math.radians(-3 * signal)
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
                print('开始直行2')
                time.sleep(7)# 直行一段时间(continue straight for a while)
                print('完成直行一段时间')

                PuppyMove['x'] = 7
                PuppyMove['yaw_rate'] = math.radians(-9 * signal)
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
                # 前面已经绕开目标物体，现在开始回来寻找直线(having circumvented the target object ahead, it's time to return and search for the straight line)
                time.sleep(2)
                print('开始3')
                area_max_times = 0
                t = time.time()
                while time.time() - t < 6:
                    if area_max > 5000:
                        
                        print('area_max',area_max)
                        break # 目标物体已经基本离开视野(the target object has mostly left the field of view)
                    time.sleep(0.1)
                print('完成寻找直线',time.time() - t)
                print('结束')
                # puppy.move_stop(servo_run_time = 100)
                # time.sleep(0.1)
                puppyStatus = PuppyStatus.NORMAL
                break
            time.sleep(0.02)
        else:
            PuppyMove['x'] = 0
            PuppyMove['yaw_rate'] = math.radians(0)
            PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
            time.sleep(0.02)

        if puppyStatusLast != puppyStatus:
            print('puppyStatus',puppyStatus)
        puppyStatusLast = puppyStatus

# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.setDaemon(True)
# th.start()

# roi = [ # [ROI, weight]
#         (0, 40,  0, 640, 0.1), 
#         (80, 120,  0, 640, 0.2), 
#         (160, 200,  0, 640, 0.7)
#        ]
roi = [ # [ROI, weight]
        (240, 280,  0, 640, 0.1), 
        (320, 360,  0, 640, 0.2), 
        (400, 440,  0, 640, 0.7)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

size = (640, 480)
def run(img):
    global line_centerx
    global __target_color, __isRunning, puppyStatus, trend, area_max
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
#     cv2.line(img, (int(img_w/2), 0), (int(img_w/2), img_h), (255, 0, 255), 2)
    
    if not __isRunning or __target_color == ():
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
            
    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0

    #将图像分割成上中下三个部分，这样处理速度会更快，更精确(Dividing the image into upper, middle, and lower parts, which will result in faster and more accurate processing)
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1       
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
        
        # area_max = 0
        areaMaxContour = 0
        for i in color_range_list:
            if i in __target_color:
                detect_color = i
                
                frame_mask = cv2.inRange(frame_lab,
                                             (color_range_list[detect_color]['min'][0],
                                              color_range_list[detect_color]['min'][1],
                                              color_range_list[detect_color]['min'][2]),
                                             (color_range_list[detect_color]['max'][0],
                                              color_range_list[detect_color]['max'][1],
                                              color_range_list[detect_color]['max'][2]))  #对原图像和掩模进行位运算(perform operation to original image and mask)
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算(opening operation)
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算(closing operation)
        #closed[:, 0:160] = 0
        #closed[:, 480:640] = 0        
        cnts = cv2.findContours(closed , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓(find out all the contours)
        cnt_large, area = getAreaMaxContour(cnts)#找到最大面积的轮廓(find out the contour with the maximal area)
        
        if cnt_large is not None:#如果轮廓不为空(if contour is not none)
            rect = cv2.minAreaRect(cnt_large)#最小外接矩形(the minimum bounding rectangle)
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点(the four vertices of the minimum bounding rectangle)
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):                
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
                
            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形(draw a rectangle formed by connecting the four points)
            
            #获取矩形的对角点(obtain the diagonal points of the rectangle)
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]            
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点(center point)
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0,0,255), -1)#画出中心点(draw out the center point)
            if n == 1:
                first_point_x = center_x
            center_.append([center_x, center_y])                        
            #按权重不同对上中下三个中心点进行求和(summing up the three central points of the top, middle, and bottom with different weights)
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]

    if puppyStatus == PuppyStatus.NORMAL or puppyStatus == PuppyStatus.FOUND_TARGET or puppyStatus == PuppyStatus.AVOID_TARGET:
        frame_lab_all = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
        frame_mask = cv2.inRange(frame_lab_all,
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
        # print('area',area)
        


    if weight_sum is not 0:
        #求最终得到的中心点(calculate the final resulting center point)
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0,255,255), -1)#画出中心点(draw the center point)
        line_centerx = int(centroid_x_sum / weight_sum)  

        if puppyStatus == PuppyStatus.NORMAL:
            try:
                if first_point_x - line_centerx < 0: trend = 'left'
                else: trend = 'right'
            except:pass
    else:
        line_centerx = -1

    return img

def image_callback(ros_image):
    # global lock
    
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customize image information to image)
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    
    frame = cv2_img.copy()
    frame_result = frame
    # with lock:
    if __isRunning:
        frame_result = run(frame)
        cv2.imshow('Frame', frame_result)
        key = cv2.waitKey(1)

    # rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tobytes()
    # ros_image.data = rgb_image
    # image_pub.publish(ros_image)

def cleanup():
    PuppyVelocityPub.publish(x=0, y=0, yaw_rate=0)
    print('is_shutdown')
if __name__ == '__main__':



    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    rospy.on_shutdown(cleanup)

    PP = rospy.get_param('/puppy_control/PuppyPose')
    PuppyPose = PP['LookDown_20deg'].copy()
    PG = rospy.get_param('/puppy_control/GaitConfig')
    GaitConfig = PG['GaitConfigFast'].copy()

    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher


    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

    rospy.sleep(0.5)
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    
    rospy.sleep(0.2)
    PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    rospy.sleep(0.2)

    th.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
        
