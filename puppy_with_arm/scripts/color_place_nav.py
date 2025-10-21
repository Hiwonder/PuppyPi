#!/home/pi/venv/bin/python3
# coding=utf8

import sys
import cv2
import math
import time
import rospy
from enum import Enum
import threading
import numpy as np
from arm_kinematics.ArmMoveIK import ArmIK
from common import Misc
from common import PID
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from puppy_control.msg import Velocity, Pose, Gait,SetServo

from std_srvs.srv import *
from actionlib_msgs.msg import GoalStatusArray

from puppy_control.srv import SetRunActionName


sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse

def cv2_image2ros(image, frame_id=''):
    image = image[:,:,::-1]
    ros_image = Image()
    header = Header(stamp=rospy.Time.now())
    header.frame_id = frame_id
    ros_image.height = image.shape[:2][0]
    ros_image.width = image.shape[:2][1]
    ros_image.encoding = 'rgb8'
    ros_image.data = image.tostring()
    ros_image.header = header
    ros_image.step = ros_image.width * 3

    return ros_image

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
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 300, the contour with the maximal area is valid to filter the interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

Bend = {'roll':math.radians(0), 'pitch':math.radians(-17), 'yaw':0.000, 'height':-10, 'x_shift':0.5, 'stance_x':0, 'stance_y':0}
Stand = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-1, 'stance_x':0, 'stance_y':0}
GaitConfig = {'overlap_time':0.15, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':3}
PuppyPose = Bend.copy()
PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}

target_color = 'red'
nav_status = False  
is_running = False      
image = None        
block_center_point = (0,0) # 物块中心点坐标(block center point coordinates)

AK = ArmIK()

class PuppyStatus(Enum):
    START = 0
    NORMAL = 1 # 正常前进中(moving forward normally)
    FOUND_TARGET = 2 # 已经发现目标物块(the target block has been found)
    PLACE = 3 # 已经发现目标物块(the target block has been found)
    STOP = 4
    END = 5

puppyStatus = PuppyStatus.START

size = (320, 240)    
img_centerx = 320 # 理论值是640/2 = 320具体值需要根据不同机器的实际运行情况微调一下(the theoretical value is 640/2 = 320. The specific value needs to be fine-tuned based on the actual operating conditions of different machines)

def move():
    global block_center_point
    global PuppyPose,PuppyPosePub
    global puppyStatus
    while not rospy.is_shutdown():
        if puppyStatus == PuppyStatus.START:
            
            PuppyPose = Bend.copy()
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
            PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                    , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
            time.sleep(1)
            puppyStatus = PuppyStatus.NORMAL
        elif puppyStatus == PuppyStatus.NORMAL:
            if block_center_point[1] > 270  and abs(block_center_point[0] - img_centerx) < 70: # 已经发现目标物块,只有物块在线上才满足(target object detected; it is considered valid only if the object is on the line)
                
                rospy.ServiceProxy('/puppy_control/go_home', Empty)()
                time.sleep(0.1)
                runActionGroup_srv('place.d6a',True) # 执行动作组(perform action group)
                puppyStatus = PuppyStatus.STOP
            else:
                value = block_center_point[0] - img_centerx
                if block_center_point[1] <= 250:
                    PuppyMove['x'] = 10
                    PuppyMove['yaw_rate'] = math.radians(0)
                elif abs(value) > 80:
                    PuppyMove['x'] = 5
                    PuppyMove['yaw_rate'] = math.radians(-11 * np.sign(value))
                elif abs(value) > 50:
                    PuppyMove['x'] = 5
                    PuppyMove['yaw_rate'] = math.radians(-5 * np.sign(value))
                elif block_center_point[1] <= 270:
                    PuppyMove['x'] = 8
                    PuppyMove['yaw_rate'] = math.radians(0)    
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
        elif puppyStatus == PuppyStatus.STOP:
            AK.setPitchRangeMoving((8.51,0,3.3),500)
            time.sleep(0.5)
            PuppyPose = Stand.copy()
            PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
            time.sleep(0.5)
th1 = threading.Thread(target=move,daemon=True)

def run():
    global image
    global block_center_point

    while not rospy.is_shutdown():
        if image is not None:
            img_copy = image.copy()
            img_h, img_w = image.shape[:2]
            frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
            frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)  
            bgr_image = cv2.cvtColor(frame_gb, cv2.COLOR_RGB2BGR) 
            frame_lab_all = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
            max_area = 0
            areaMaxContour_max = 0
            frame_mask = cv2.inRange(frame_lab_all,
                                            (color_range_list[target_color]['min'][0],
                                            color_range_list[target_color]['min'][1],
                                            color_range_list[target_color]['min'][2]),
                                            (color_range_list[target_color]['max'][0],
                                            color_range_list[target_color]['max'][1],
                                            color_range_list[target_color]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out contour)
            areaMaxContour, area_max = getAreaMaxContour(contours)  #找出最大轮廓(find out the contour with the maximal area)
            if areaMaxContour is not None:
                if area_max > max_area:#找最大面积(find the maximal area)
                    max_area = area_max
                    areaMaxContour_max = areaMaxContour
            if max_area > 200:
                ((centerX, centerY), radius) = cv2.minEnclosingCircle(areaMaxContour_max)  # 获取最小外接圆(obtain minimum circumcircle)
                centerX = int(Misc.map(centerX, 0, size[0], 0, img_w))
                centerY = int(Misc.map(centerY, 0, size[1], 0, img_h))
                radius = int(Misc.map(radius, 0, size[0], 0, img_w))  
                block_center_point = (centerX,centerY)
                #print(block_center_point)
                cv2.circle(bgr_image, (centerX, centerY), radius, (0,255,255), 2)#画圆(draw circle)
            result_publisher.publish(cv2_image2ros(cv2.resize(bgr_image, (640, 480)), 'navigation_status_listener'))

th = threading.Thread(target=run,daemon=True)     

def image_callback(ros_image):
    global image

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) # 原始 RGB 画面(original RGB image)

def goal_status_callback(msg):
    global nav_status
    
    if len(msg.status_list) > 0:
        status = msg.status_list[0].status
        if status == 3:  # 3表示目标已经到达(3 means the target has arrived)
            if not nav_status:
                rospy.loginfo("target") 
                th.start()
                th1.start()
            nav_status = True

if __name__ == '__main__':
   
    rospy.init_node('navigation_status_listener')
    color_range_list = rospy.get_param('/lab_config_manager/color_range_list', {})
    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
    rospy.Subscriber('/move_base/status', GoalStatusArray, goal_status_callback)
    rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)
    result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)  # 图像处理结果发布,# 使用~可以自动加上前缀名称(image processing result published. use "~" to automatically add the prefix name)
    time.sleep(0.3)
    PuppyPose = Stand.copy()
    PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 500)
    time.sleep(0.5)
    
    AK.setPitchRangeMoving((8.51,0,3.3),500)
    time.sleep(0.5)
    Debug = False
    if Debug:
        time.sleep(0.5)
        th.start()
        th1.start()
       
    rospy.spin()

