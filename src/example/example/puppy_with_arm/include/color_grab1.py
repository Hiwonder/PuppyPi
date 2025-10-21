#!/usr/bin/python3
# coding=utf8
# 第11章 ROS机器狗结合机械臂课程\第4课 自主识别夹取(11.ROS Robot Combine Robotic Arm Course\Lesson 4 Auto Recognition Gripping)
import sys
import cv2
import time
import math
import threading
import numpy as np
from enum import Enum
from sdk.ArmMoveIK import ArmIK
from sdk import Misc
from sdk import Camera
from sdk import yaml_handle

import gpiod


sys.path.append('/home/ubuntu/software/puppypi_control/') 
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup
from puppy_kinematics import HiwonderPuppy, PWMServoParams

puppy = HiwonderPuppy(setServoPulse = setServoPulse, servoParams = PWMServoParams(), dof = '8')

Stand = {'roll':math.radians(0.0), 'pitch':math.radians(0.0), 'yaw':0.000, 'height':-10.0, 'x_shift':-0.5, 'stance_x':0.0, 'stance_y':0.0}
Bend = {'roll':math.radians(0.0), 'pitch':math.radians(-17.0), 'yaw':0.000, 'height':-10.0, 'x_shift':0.5, 'stance_x':0.0, 'stance_y':0.0}

PuppyPose = Stand.copy()
# stance_x：4条腿在x轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the X-axis, measured in centimeters)
# stance_y：4条腿在y轴上额外分开的距离，单位cm(the distance extra apart for each of the four legs on the Y-axis, measured in centimeters)
# x_shift: 4条腿在x轴上同向移动的距离，越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡，单位cm(the distance traveled by the four legs along the x-axis determines the degree of forward or backward tilt during walking: smaller distances lead to more forward tilt, while larger distances result in more backward tilt. Adjusting the x_shift parameter can help maintain balance during the dog's movement, measured in centimeters)
# height： 狗的高度，脚尖到大腿转动轴的垂直距离，单位cm(the height of the dog, measured from the toe to the axis  of rotation of the thigh, is in centimeters)
# pitch： 狗身体的俯仰角，单位弧度(the pitch angle of the dog's body, measured in radians)


GaitConfig = {'overlap_time':0.15, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':3}
# GaitConfig = GaitConfigFast.copy()
# overlap_time:4脚全部着地的时间，单位秒(the time when all four legs touch the ground, measured in seconds)
# swing_time：2脚离地时间，单位秒(the time duration when legs are off the ground, measured in second)
# clearance_time：前后脚间隔时间，单位秒(the time interval between the front and rear legs, measured in seconds)
# z_clearance：走路时，脚抬高的距离，单位cm(the distance the paw needs to be raised during walking, measured in centimeters)

PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}
# x:直行控制，  前进方向为正方向，单位cm/s(straightforward control, with the forward direction as the positive direction, measured in centimeters per second)
# y:侧移控制，左侧方向为正方向，单位cm/s，目前无此功能(lateral movement control, with the left direction as the positive direction, measured in cm/s. currently, this function is not available)
# yaw_rate：转弯控制，逆时针方向为正方向，单位rad/s(turning control, with counterclockwise direction as the positive direction, measured in rad/s)


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

key1_pin = 25 # KEY1短按启动程序(KEY1 short press to start the program)
key2_pin = 23 # KEY2短按停止程序(KEY2 short press to stop the program)
chip = gpiod.chip("gpiochip0")

key1 = chip.get_line(key1_pin)
config = gpiod.line_request()
config.consumer = "key1"
config.request_type = gpiod.line_request.DIRECTION_INPUT
config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
key1.request(config)

key2 = chip.get_line(key2_pin)
config = gpiod.line_request()
config.consumer = "key2"
config.request_type = gpiod.line_request.DIRECTION_INPUT
config.flags = gpiod.line_request.FLAG_BIAS_PULL_UP
key2.request(config)

block_color = 'None'
block_center_point = (0,0) # 物块中心点坐标(block center point coordinates)
AK = ArmIK()
color_list = []

class PuppyStatus(Enum):
    START = 0
    NORMAL = 1 # 正常前进中(moving forward normally)
    FOUND_TARGET = 3 # 已经发现目标物块(target block detected)
    PLACE = 4 # 已经发现目标物块(target block has been found)
    STOP = 10
    END = 20            

puppyStatus = PuppyStatus.STOP
puppyStatusLast = PuppyStatus.END


line_centerx = -1 # 线条中心坐标位置(line center coordinate position)
img_centerx = 320 # 理论值是640/2 = 320具体值需要根据不同机器的实际运行情况微调一下(the theoretical value is 640/2 = 320. The specific value needs to be fine-tuned based on the actual operation of different machines)
def move():
    global line_centerx, puppyStatus, puppyStatusLast, block_center_point
    global block_color

    while True:
        while(puppyStatus == PuppyStatus.START) :  # 启动阶段，初始化姿态(startup stage, initialize the posture)
            
            puppy.move_stop(servo_run_time = 500)
            PuppyPose = Bend.copy()
            puppy.stance_config(stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
            time.sleep(0.5)
            
            puppyStatus = PuppyStatus.NORMAL
            break

        while(puppyStatus == PuppyStatus.NORMAL) : # 
            
            if block_center_point[1] > 355 and block_center_point[1] < 370 and block_color == 'green' and abs(block_center_point[0] - img_centerx) < 50: # 已经发现目标物块,只有物块在线上才满足(target object has been found, it is considered valid only when the block is on the line)
                puppyStatus = PuppyStatus.FOUND_TARGET
                puppy.move_stop(servo_run_time = 500)
                time.sleep(0.5)
                break
            value = block_center_point[0] - img_centerx
            if block_center_point[1] <= 300:
                PuppyMove['x'] = 10.0
                PuppyMove['yaw_rate'] = math.radians(0.0)
            elif abs(value) > 80:
                PuppyMove['x'] = 5
                PuppyMove['yaw_rate'] = math.radians(-11.0 * np.sign(value))
            elif abs(value) > 50.0:
                PuppyMove['x'] = 5.0
                PuppyMove['yaw_rate'] = math.radians(-5.0 * np.sign(value))
            elif block_center_point[1] <= 355.0:
                PuppyMove['x'] = 8.0
                PuppyMove['yaw_rate'] = math.radians(0.0)
            elif block_center_point[1] >= 370.0:
                PuppyMove['x'] = -5.0
                PuppyMove['yaw_rate'] = math.radians(0.0)
            puppy.move(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
            break

        while(puppyStatus == PuppyStatus.FOUND_TARGET) : # 发现物块(find the block)
            
            runActionGroup('grab.d6a',True) # 执行动作组(perform action group)
            PuppyPose = Stand.copy()
            puppy.stance_config(stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']+1), PuppyPose['pitch'], PuppyPose['roll'])
            time.sleep(0.5)

            PuppyMove['x'] = 10.0
            PuppyMove['yaw_rate'] = math.radians(0.0)
            puppy.move(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
            time.sleep(1)

            PuppyMove['x'] = 5
            PuppyMove['yaw_rate'] = math.radians(20.0)
            puppy.move(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
            time.sleep(3)

            puppy.move_stop(servo_run_time = 500)
            time.sleep(0.5)
            PuppyPose = Bend.copy()
            puppy.stance_config(stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
            time.sleep(0.5)
            puppyStatus = PuppyStatus.PLACE
            break

        while(puppyStatus == PuppyStatus.PLACE) : # 放置物块(place block)
            
            if block_center_point[1] > 270  and block_color == 'red' and abs(block_center_point[0] - img_centerx) < 70: # 已经发现目标物块,只有物块在线上才满足(target object detected; it is considered valid only when the block is on the line)
                
                puppy.move_stop(servo_run_time = 100)
                time.sleep(0.1)
                runActionGroup('place.d6a',True) # 执行动作组(perform action group)
                puppyStatus = PuppyStatus.STOP
                break
            value = block_center_point[0] - img_centerx
            if block_center_point[1] <= 230:
                PuppyMove['x'] = 10.0
                PuppyMove['yaw_rate'] = math.radians(0.0)
            elif abs(value) > 80.0:
                PuppyMove['x'] = 5.0
                PuppyMove['yaw_rate'] = math.radians(-11.0 * np.sign(value))
            elif abs(value) > 50.0:
                PuppyMove['x'] = 5.0
                PuppyMove['yaw_rate'] = math.radians(-5.0 * np.sign(value))
            elif block_center_point[1] <= 270:
                PuppyMove['x'] = 8.0
                PuppyMove['yaw_rate'] = math.radians(0.0)
            
            puppy.move(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate = PuppyMove['yaw_rate'])
            break
                
            

        while(puppyStatus == PuppyStatus.STOP) :
            puppy.move_stop(servo_run_time = 500)
            PuppyPose = Stand.copy()
            puppy.stance_config(stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
            time.sleep(0.5)
            break
        

        time.sleep(0.02)

        if puppyStatusLast != puppyStatus:
            print('puppyStatus',puppyStatus)
        puppyStatusLast = puppyStatus
# 运行子线程(run sub-thread)
th = threading.Thread(target=move)
th.daemon = True




size = (640, 480)

lab_data = None

def load_config():
    global lab_data

    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)['color_range_list']

load_config()

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
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰(only when the area is greater than 300, the contour with the maximal area is valid to filter the interference)
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓(return the contour with the maximal area)

def run(img):
    
    global puppyStatus, block_center_point
    global block_color
    global color_list
    global Debug
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
                       
    frame_lab_all = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间(convert the image to LAB space)
    max_area = 0
    color_area_max = None    
    areaMaxContour_max = 0

    for i in lab_data:
        if i in ['red']:
            frame_mask = cv2.inRange(frame_lab_all,
                                            (lab_data[i]['min'][0],
                                            lab_data[i]['min'][1],
                                            lab_data[i]['min'][2]),
                                            (lab_data[i]['max'][0],
                                            lab_data[i]['max'][1],
                                            lab_data[i]['max'][2]))  #对原图像和掩模进行位运算(perform bitwise operation to original image and mask)
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀(corrosion)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))) #膨胀(dilation)
            
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  #找出轮廓(find out the contour)
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
        block_center_point = (centerX,centerY)   
        
        if Debug:
            print("block_center_x",block_center_point[0])   
            print("block_center_y",block_center_point[1])     
        cv2.circle(img, (centerX, centerY), radius, (0,255,255), 2)#画圆(draw circle)
        
        if color_area_max == 'red':  #红色最大(red is the maximal area)
            color = 1
        elif color_area_max == 'green':  #绿色最大(green is the maximal area)
            color = 2
        elif color_area_max == 'blue':  #蓝色最大(blue is the maximal area)
            color = 3
        else:
            color = 0
        color_list.append(color)

        if len(color_list) == 3:  #多次判断(multiple judgements)
            # 取平均值(take average value)
            color = int(round(np.mean(np.array(color_list))))
            color_list = []
            if color == 1:
                block_color = 'red'
                
            elif color == 2:
                block_color = 'green'
                
            elif color == 3:
                block_color = 'blue'
                
            else:
                block_color = 'None'
                
    else:
        block_color = 'None'

    return img

def stance(x = 0.0, y = 0.0, z = -11.0, x_shift = 2.0):# 单位cm(unit: cm)
    # x_shift越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡(the smaller the x_shift, the more forward-leaning the walk; the larger, the more backward-leaning. Adjusting x_shift can balance the robot dog's gait)
    return np.array([
                        [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
                        [y, y, y, y],
                        [z, z, z, z],
                    ])#此array的组合方式不要去改变(please refrain from altering the combination of this array)

if __name__ == '__main__':

    puppy.stance_config(stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
    puppy.gait_config(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                            , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
    # overlap_time:4脚全部着地的时间，swing_time：2脚离地时间，z_clearance：走路时，脚抬高的距离(overlap_time: The time when all four feet touch the ground, swing_time: The time when two feet are off the ground, z_clearance: The height at which the feet are lifted during walking)

    puppy.start() # 启动(start)
    puppy.move_stop(servo_run_time = 500)

    AK.setPitchRangeMoving((8.51,0,3.3),500)
    setServoPulse(9,1500,500)
    time.sleep(0.5)
    
    
    Debug = False
    
    if Debug:
        PuppyPose = Bend.copy()
        puppy.stance_config(stance(PuppyPose['stance_x'],PuppyPose['stance_y'],PuppyPose['height'],PuppyPose['x_shift']), PuppyPose['pitch'], PuppyPose['roll'])
        time.sleep(0.5)
        
    else:
        th.start() 
    #   开启机器狗移动线程(start the moving thread of robot dog)
    #   如果只是想看摄像头画面，调试画面，不想让机器人移动，可以把Debug设置为True，同时不要去按按钮(if you only want to view the camera feed and debug the image without moving the robot, you can set Debug to True, and avoid pressing any buttons)
    #   等调试完再把Debug设置为False，观察实际运行情况(once debugging is complete, set Debug to False, and observe the actual operation)

    my_camera = Camera.Camera()
    my_camera.camera_open()
    
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)

        if key1.get_value() == 0:
            time.sleep(0.05)
            if key1.get_value() == 0:
                puppyStatus = PuppyStatus.START
        if key2.get_value() == 0:
            time.sleep(0.05)
            if key2.get_value() == 0:
                puppyStatus = PuppyStatus.STOP
                stopActionGroup()
    my_camera.camera_close()
    cv2.destroyAllWindows()

