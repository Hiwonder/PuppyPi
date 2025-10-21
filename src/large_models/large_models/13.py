#!/usr/bin/python3
# coding=utf8
# color_tracking/scripts/puppy_control.py

import sys
import cv2
import time
import math
import threading
import numpy as np
from enum import Enum

# 导入自定义模块
from sdk.ArmMoveIK import ArmIK
from sdk import Misc, yaml_handle

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from large_models_msgs.srv import SetColor
sys.path.append('/home/ubuntu/software/puppypi_control/') 
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup
from puppy_kinematics import HiwonderPuppy, PWMServoParams

# 定义机器人状态枚举
class PuppyStatus(Enum):
    IDLE = 0          # 空闲状态，等待颜色设置或手动启动
    START = 1         # 启动阶段，初始化姿态
    NORMAL = 2        # 正常前进中
    FOUND_TARGET = 3  # 已经发现目标物块
    ADJUSTING_YAW = 4 # 微调方向（yaw）状态
    ADJUSTING_X = 5   # 微调位置（x）状态
    STOP = 10         # 停止状态，禁用部分服务

class PuppyController:
    def __init__(self):
        # 初始化机器狗
        self.puppy = HiwonderPuppy(setServoPulse=setServoPulse, servoParams=PWMServoParams(), dof='8')
        
        # 定义姿态
        self.stand_pose = {
            'roll': math.radians(0.0),
            'pitch': math.radians(0.0),
            'yaw': 0.000,
            'height': -10.0,
            'x_shift': -1.0,
            'stance_x': 0.0,
            'stance_y': 0.0
        }
        self.bend_pose = {
            'roll': math.radians(0.0),
            'pitch': math.radians(-17.0),
            'yaw': 0.000,
            'height': -10.0,
            'x_shift': 0.5,
            'stance_x': 0.0,
            'stance_y': 0.0
        }
        
        # 当前姿态
        self.current_pose = self.stand_pose.copy()
        
        # 步态配置
        self.gait_config = {
            'overlap_time': 0.15,
            'swing_time': 0.15,
            'clearance_time': 0.0,
            'z_clearance': 3
        }
        
        # 移动控制参数
        self.puppy_move = {
            'x': 0,
            'y': 0,
            'yaw_rate': 0
        }
        
        # 反解控制
        self.arm_ik = ArmIK()
        self.color_list = []
        
        # 初始化状态
        self.status = PuppyStatus.IDLE
        self.status_last = PuppyStatus.STOP  # 初始为 STOP 状态
        
        # 图像处理相关变量
        self.img_centerx = 320
        self.block_color = 'None'
        self.block_center_point = (0, 0)
        self.radius = 0
        self.size = (640, 480)
        self.lab_data = None
        self.MIN_RADIUS = 129
        self.MAX_RADIUS = 140
        self.Debug = False
        
        # 颜色跟踪相关变量
        self.color_tracking_enabled = False
        self.target_color = 'None'
        self.color_list = []
        
        # 调整子状态
        self.adjusting_substatus = None  # None, 'YAW', 'X'
        
        # 多帧确认变量
        self.adjusting_counter_yaw = 0
        self.required_frames_yaw = 3  # 需要连续 3 帧满足条件进行方向调整
        self.adjusting_counter_x = 0
        self.required_frames_x = 3    # 需要连续 3 帧满足条件进行位置调整
        
        # 定义更严格的y坐标阈值
        self.y_close_lower = 355  # 增加下限，确保距离足够近
        self.y_close_upper = 390
        
        # 线程锁
        self.status_lock = threading.Lock()
        
        # 加载颜色配置
        self.load_config()
        
        # 初始化步态和姿态
        self.initialize_puppy()
        
        # 启动机器狗
        self.puppy.start()
        self.puppy.move_stop(servo_run_time=500)
        
        # 设置初始姿态
        self.arm_ik.setPitchRangeMoving((8.51, 0, 3.3), 500)
        setServoPulse(9, 1500, 500)
        time.sleep(0.5)
        
        if self.Debug:
            self.current_pose = self.bend_pose.copy()
            self.puppy.stance_config(
                self.stance(self.current_pose['stance_x'], self.current_pose['stance_y'], self.current_pose['height'], self.current_pose['x_shift']),
                self.current_pose['pitch'], self.current_pose['roll']
            )
            time.sleep(0.5)
        else:
            # 开启移动控制线程
            self.move_thread = threading.Thread(target=self.move)
            self.move_thread.daemon = True
            self.move_thread.start()

    def load_config(self):
        """加载颜色配置文件"""
        self.lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)['color_range_list']
    
    def stance(self, x=0.0, y=0.0, z=-11.0, x_shift=2.0):
        """
        定义姿态配置函数
        x_shift越小，走路越前倾，越大越后仰
        """
        return np.array([
            [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
            [y, y, y, y],
            [z, z, z, z],
        ])  # 此array的组合方式不要去改变
    
    def initialize_puppy(self):
        """初始化机器狗的姿态和步态配置"""
        self.puppy.stance_config(
            self.stance(self.current_pose['stance_x'], self.current_pose['stance_y'], self.current_pose['height'], self.current_pose['x_shift']),
            self.current_pose['pitch'], self.current_pose['roll']
        )
        self.puppy.gait_config(
            overlap_time=self.gait_config['overlap_time'],
            swing_time=self.gait_config['swing_time'],
            clearance_time=self.gait_config['clearance_time'],
            z_clearance=self.gait_config['z_clearance']
        )
    
    def get_area_max_contour(self, contours):
        """找出面积最大的轮廓"""
        contour_area_max = 0
        area_max_contour = None

        for c in contours:
            contour_area = math.fabs(cv2.contourArea(c))
            if contour_area > contour_area_max and contour_area >= 5:
                contour_area_max = contour_area
                area_max_contour = c

        return area_max_contour, contour_area_max
    
    def process_image(self, img):
        """
        图像处理函数
        进行颜色检测，找到目标物体的位置和大小
        """
        if not self.color_tracking_enabled:
            return img  # 如果颜色跟踪未启用，直接返回原图

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   

        frame_lab_all = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        if self.target_color not in self.lab_data:
            print(f"目标颜色 {self.target_color} 不在颜色配置中！")
            return img

        # 获取目标颜色的范围
        color_range = self.lab_data[self.target_color]
        frame_mask = cv2.inRange(
            frame_lab_all,
            (color_range['min'][0], color_range['min'][1], color_range['min'][2]),
            (color_range['max'][0], color_range['max'][1], color_range['max'][2])
        )
        
        # 腐蚀和膨胀操作
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        eroded = cv2.erode(frame_mask, kernel)
        dilated = cv2.dilate(eroded, kernel)

        # 找出轮廓
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        area_max_contour, area_max = self.get_area_max_contour(contours)

        if area_max_contour is not None and area_max >= 5:
            ((centerX, centerY), radius_temp) = cv2.minEnclosingCircle(area_max_contour)
            print(f"映射前的 centerX: {centerX}, centerY: {centerY}")
            centerX = int(Misc.map(centerX, 0, self.size[0], 0, img_w))
            centerY = int(Misc.map(centerY, 0, self.size[1], 0, img_h))
            radius = int(Misc.map(radius_temp, 0, self.size[0], 0, img_w))  
            print(f"映射后的 centerX: {centerX}, centerY: {centerY}, radius: {radius}")
            self.block_center_point = (centerX, centerY)
            self.radius = radius

            # 打印目标位置和误差
            print(f"检测到目标位置: x={self.block_center_point[0]}, y={self.block_center_point[1]}, 半径={self.radius}")
            print(f"误差: x偏差={abs(self.block_center_point[0] - self.img_centerx)}")

            # 绘制最小外接圆
            cv2.circle(img, self.block_center_point, self.radius, (0, 255, 255), 2)

            # 判断是否为目标颜色
            color = 1 if self.MIN_RADIUS <= self.radius <= self.MAX_RADIUS else 0
            self.color_list.append(color)

            # 多次判断取平均
            if len(self.color_list) == 3:
                color_avg = int(round(np.mean(np.array(self.color_list))))
                self.color_list = []
                self.block_color = self.target_color if color_avg == 1 else 'None'

            # 稳定条件判断
            # 根据图像坐标系，y 越大，目标越近
            if (self.y_close_lower < self.block_center_point[1] < self.y_close_upper and 
                self.block_color == self.target_color 
                and abs(self.block_center_point[0] - self.img_centerx) < 50
                ):
                # 满足条件，增加 y 方向调整计数器
                self.adjusting_counter_yaw += 1
                print(f"微调计数器（方向）: {self.adjusting_counter_yaw}")
                if self.adjusting_counter_yaw >= self.required_frames_yaw:
                    # 满足多帧条件，设置为 ADJUSTING_YAW
                    print("满足多帧条件，设置为 ADJUSTING_YAW")
                    try:
                        self.puppy.move_stop(servo_run_time=500)  # 停止移动
                    except Exception as e:
                        print(f"停止移动时出错: {e}")
                    self.status = PuppyStatus.ADJUSTING_YAW
                    self.adjusting_counter_yaw = 0
                    time.sleep(0.5)
            else:
                # 如果不满足条件，重置计数器
                self.adjusting_counter_yaw = 0
                self.block_color = 'None'
                self.radius = 0

        else:
            self.block_color = 'None'
            self.radius = 0

        # 在 STOP 状态下，检查是否检测到颜色以重新启动
        if self.status == PuppyStatus.STOP and self.block_color == self.target_color:
            print("在 STOP 状态下检测到目标颜色，自动重新启动颜色跟踪模式")
            with self.status_lock:
                self.status = PuppyStatus.START
                self.color_tracking_enabled = True  # 重新启用颜色跟踪

        # 绘制微调触发区域
        cv2.rectangle(img, (0, self.y_close_lower), (img_w, self.y_close_upper), (255, 0, 0), 2)

        # 绘制目标物体中心
        if self.block_center_point != (0, 0):
            cv2.circle(img, self.block_center_point, 5, (0, 255, 0), -1)

        # 绘制图像中心线
        cv2.line(img, (self.img_centerx, 0), (self.img_centerx, img_h), (0, 255, 255), 2)

        # 绘制方向调整指示箭头（示例）
        if self.status == PuppyStatus.ADJUSTING_YAW:
            if self.puppy_move['yaw_rate'] > 0:
                cv2.arrowedLine(img, (self.img_centerx, self.block_center_point[1]), 
                                (self.img_centerx + 30, self.block_center_point[1]), 
                                (0, 0, 255), 2)
            elif self.puppy_move['yaw_rate'] < 0:
                cv2.arrowedLine(img, (self.img_centerx, self.block_center_point[1]), 
                                (self.img_centerx - 30, self.block_center_point[1]), 
                                (0, 0, 255), 2)

        # 绘制位置调整指示箭头（示例）
        if self.status == PuppyStatus.ADJUSTING_X:
            if self.puppy_move['x'] > 0:
                cv2.arrowedLine(img, (self.block_center_point[0], self.block_center_point[1]), 
                                (self.block_center_point[0], self.block_center_point[1] - 30), 
                                (0, 255, 0), 2)
            elif self.puppy_move['x'] < 0:
                cv2.arrowedLine(img, (self.block_center_point[0], self.block_center_point[1]), 
                                (self.block_center_point[0], self.block_center_point[1] + 30), 
                                (0, 255, 0), 2)

        return img
    
    def is_within_clamp_range(self, y, x_error):
        """
        判断目标是否在夹取范围内。
        可以根据具体需求调整y的范围和x误差的阈值。
        """
        y_close = 355 < y <= 370  # 确保y坐标在预定范围内
        x_close = abs(x_error) < 30  # x误差在允许范围内
        print(f"判断夹取范围: y_close={y_close}, x_close={x_close}")
        return y_close and x_close
    
    def adjust_for_fine_tuning(self, x_error):
        """
        进行微调操作，调整机器狗的位置或方向。
        """
        if abs(x_error) > 10:
            yaw_rate = math.radians(-2.0 * np.sign(x_error))
            try:
                self.puppy.move(
                    x=0.0,
                    y=0.0,
                    yaw_rate=yaw_rate
                )
                print(f"进行微调: yaw_rate={yaw_rate}")
            except Exception as e:
                print(f"微调时出错: {e}")
            time.sleep(0.5)
            self.puppy.move_stop(servo_run_time=500)
            print("完成微调，停止移动")
    
    def is_within_range(self):
        """判断目标是否在允许的范围内"""
        y_in_range = 355 < self.block_center_point[1] < 370
        x_in_range = abs(self.block_center_point[0] - self.img_centerx) < 30
        color_correct = self.block_color == self.target_color
        print(f"判断目标是否在范围内: y_in_range={y_in_range}, x_in_range={x_in_range}, color_correct={color_correct}")
        return y_in_range and x_in_range and color_correct
    
    def move(self):
        """移动控制线程"""
        while rclpy.ok():
            if self.status == PuppyStatus.START:
                try:
                    self.puppy.move_stop(servo_run_time=500)
                except Exception as e:
                    print(f"停止移动时出错: {e}")
                self.current_pose = self.bend_pose.copy()
                self.puppy.stance_config(
                    self.stance(self.current_pose['stance_x'], self.current_pose['stance_y'], self.current_pose['height'], self.current_pose['x_shift']),
                    self.current_pose['pitch'], self.current_pose['roll']
                )
                time.sleep(0.5)

                self.status = PuppyStatus.NORMAL
                print("进入 NORMAL 状态，开始颜色跟踪")

            elif self.status == PuppyStatus.NORMAL:
                if self.color_tracking_enabled:
                    value = self.block_center_point[0] - self.img_centerx
                    y = self.block_center_point[1]
                    print(f"NORMAL 状态: y={y}, value={value}")

                    # 首先检查是否在夹取范围内
                    if self.is_within_clamp_range(y, value):
                        print("满足夹取条件，进入 FOUND_TARGET 状态")
                        self.status = PuppyStatus.FOUND_TARGET
                        # 停止移动
                        try:
                            self.puppy.move_stop(servo_run_time=500)
                            print("满足夹取条件，已停止移动")
                        except Exception as e:
                            print(f"停止移动时出错: {e}")
                    else:
                        # 初始化移动指令
                        move_x = 0.0
                        yaw_rate = 0.0

                        # 前进控制，根据y坐标决定是否前进
                        if y < self.y_close_lower:
                            # 目标较远，前进
                            move_x += 8.0
                            print("目标较远，前进")
                        else:
                            # y >= y_close_lower，目标接近但未进入夹取范围，不再前进
                            move_x = 0.0
                            print("目标接近但未进入夹取范围，停止前进")

                        # 转向控制
                        if abs(value) > 80:
                            # 目标偏离较大，快速转向
                            yaw_rate += math.radians(-8.0 * np.sign(value))
                            print(f"目标偏离较大，快速转向: yaw_rate={yaw_rate}")
                        elif abs(value) > 50.0:
                            # 目标偏离中等，缓慢转向
                            yaw_rate += math.radians(-5.0 * np.sign(value))
                            print(f"目标偏离中等，缓慢转向: yaw_rate={yaw_rate}")

                        # 更新移动参数
                        self.puppy_move['x'] = move_x
                        self.puppy_move['yaw_rate'] = yaw_rate

                        # 执行移动指令
                        try:
                            self.puppy.move(
                                x=self.puppy_move['x'],
                                y=self.puppy_move['y'],
                                yaw_rate=self.puppy_move['yaw_rate']
                            )
                            print(f"执行移动指令: x={self.puppy_move['x']} cm, yaw_rate={self.puppy_move['yaw_rate']} radians")
                        except Exception as e:
                            print(f"移动指令执行时出错: {e}")

                        # 检查是否满足夹取条件
                        if self.is_within_range():
                            print("满足夹取条件，进入 FOUND_TARGET 状态")
                            self.status = PuppyStatus.FOUND_TARGET
                            # 停止移动
                            try:
                                self.puppy.move_stop(servo_run_time=500)
                                print("满足夹取条件，已停止移动")
                            except Exception as e:
                                print(f"停止移动时出错: {e}")
                        else:
                            # 微调条件
                            if self.y_close_lower < y < self.y_close_upper and abs(value) < 30:
                                # 满足微调条件，增加微调计数器
                                self.adjusting_counter_yaw += 1
                                print(f"微调计数器（方向）: {self.adjusting_counter_yaw}")
                                if self.adjusting_counter_yaw >= self.required_frames_yaw:
                                    # 满足多帧条件，设置为 ADJUSTING_YAW
                                    self.status = PuppyStatus.ADJUSTING_YAW
                                    self.adjusting_counter_yaw = 0
                                    print("进入 ADJUSTING_YAW 状态，进行方向微调")

                else:
                    try:
                        self.puppy.move_stop(servo_run_time=100)
                        print("颜色跟踪未启用，停止移动")
                    except Exception as e:
                        print(f"停止移动时出错: {e}")
                    time.sleep(0.1)  # 逐渐停止

            elif self.status == PuppyStatus.ADJUSTING_YAW:
                try:
                    # 调整方向（yaw），直到对齐
                    error_x = self.block_center_point[0] - self.img_centerx
                    yaw_rate = 0.0

                    if error_x < -10:
                        yaw_rate = math.radians(5.0)  # 向右转
                        print("微调方向: 向右转")
                    elif error_x > 10:
                        yaw_rate = math.radians(-5.0)  # 向左转
                        print("微调方向: 向左转")

                    # 发送移动指令，仅调整方向
                    self.puppy_move['yaw_rate'] = yaw_rate
                    self.puppy_move['x'] = 0  # 保持原地

                    if yaw_rate != 0.0:
                        try:
                            self.puppy.move(
                                x=self.puppy_move['x'],
                                y=0,
                                yaw_rate=self.puppy_move['yaw_rate']
                            )
                            print(f"执行方向微调：yaw_rate 调整为 {self.puppy_move['yaw_rate']} radians")
                        except Exception as e:
                            print(f"移动指令执行时出错: {e}")

                    time.sleep(0.6)  # 移动0.6秒

                    # 停止移动并暂停
                    self.puppy.move_stop(servo_run_time=510)
                    print("执行方向微调：停止移动")
                    time.sleep(0.6)  # 暂停0.6秒

                    # 检查是否方向对齐
                    if abs(error_x) <= 10:
                        # 确保y坐标在近距离范围内
                        y = self.block_center_point[1]
                        if self.y_close_lower < y < self.y_close_upper:
                            print("方向对齐完成，进入 ADJUSTING_X 状态，进行位置微调")
                            self.status = PuppyStatus.ADJUSTING_X
                        else:
                            print("方向对齐，但目标物体不够近，继续前进")
                            self.status = PuppyStatus.NORMAL
                    else:
                        print("方向未完全对齐，继续方向微调")
                except Exception as e:
                    print(f"方向微调时出错: {e}")
                    # 发生异常时，返回 NORMAL 状态以重新尝试
                    with self.status_lock:
                        self.status = PuppyStatus.NORMAL
                    print("由于异常，返回 NORMAL 状态")

            elif self.status == PuppyStatus.ADJUSTING_X:
                try:
                    # 调整位置（x），直到对齐
                    error_y = self.block_center_point[1] - 365  # 目标位置中间点
                    move_x = 0.0

                    if error_y < -10:
                        move_x = 5.0  # 向前微调
                        print("微调位置: 向前移动")
                    elif error_y > 10:
                        move_x = -5.0  # 向后微调
                        print("微调位置: 向后移动")

                    # 仅在目标物体非常接近时调整x轴
                    if self.y_close_lower < self.block_center_point[1] < self.y_close_upper:
                        # 发送移动指令，仅调整位置
                        self.puppy_move['x'] = move_x
                        self.puppy_move['yaw_rate'] = 0.0  # 保持方向不变

                        if move_x != 0.0:
                            try:
                                self.puppy.move(
                                    x=self.puppy_move['x'],
                                    y=0,
                                    yaw_rate=self.puppy_move['yaw_rate']
                                )
                                print(f"执行位置微调：x 移动 {self.puppy_move['x']} cm")
                            except Exception as e:
                                print(f"移动指令执行时出错: {e}")

                        time.sleep(0.6)  # 移动0.6秒

                        # 停止移动并暂停
                        self.puppy.move_stop(servo_run_time=510)
                        print("执行位置微调：停止移动")
                        time.sleep(0.51)  # 暂停0.51秒

                        # 检查是否位置对齐
                        if abs(error_y) <= 10:
                            print("位置对齐完成，进入 FOUND_TARGET 状态，准备夹取")
                            self.status = PuppyStatus.FOUND_TARGET
                        else:
                            print("位置未完全对齐，继续位置微调")
                    else:
                        print("目标物体不够近，跳过位置微调")
                        # 如果目标不够近，返回 NORMAL 状态以继续前进
                        self.status = PuppyStatus.NORMAL
                except Exception as e:
                    print(f"位置微调时出错: {e}")
                    # 发生异常时，返回 NORMAL 状态以重新尝试
                    with self.status_lock:
                        self.status = PuppyStatus.NORMAL
                    print("由于异常，返回 NORMAL 状态")

            elif self.status == PuppyStatus.FOUND_TARGET:
                time.sleep(2)
                try:
                    runActionGroup('Clamping.d6a', True)  # 执行动作组，夹取物块
                    self.current_pose = self.stand_pose.copy()
                    self.puppy.stance_config(
                        self.stance(self.current_pose['stance_x'], self.current_pose['stance_y'], self.current_pose['height'], self.current_pose['x_shift'] + 1),
                        self.current_pose['pitch'], self.current_pose['roll']
                    )
                    time.sleep(0.5)
                    # 执行放置动作组
                    runActionGroup('place1.d6a', True)  # 执行放置动作组
                    time.sleep(1)  # 等待一段时间
                    runActionGroup('look_down.d6a', True)  # 执行向下看的动作组
                    time.sleep(1)  # 等待一段时间

                    # 切换到 STOP 状态，停止颜色跟踪
                    with self.status_lock:
                        self.status = PuppyStatus.STOP
                        self.color_tracking_enabled = False
                    print("完成目标物块处理，进入 STOP 状态，停止颜色跟踪")
                except Exception as e:
                    print(f"执行 FOUND_TARGET 动作时出错: {e}")
                    # 发生异常时，返回 NORMAL 状态以重新尝试
                    with self.status_lock:
                        self.status = PuppyStatus.NORMAL
                    print("由于异常，返回 NORMAL 状态")

            elif self.status == PuppyStatus.STOP:
                try:
                    self.puppy.move_stop(servo_run_time=500)  # 减小servo_run_time以加快停止
                    print("STOP 状态：停止移动")
                except Exception as e:
                    print(f"停止移动时出错: {e}")
                self.current_pose = self.stand_pose.copy()
                self.puppy.stance_config(
                    self.stance(self.current_pose['stance_x'], self.current_pose['stance_y'], self.current_pose['height'], self.current_pose['x_shift']),
                    self.current_pose['pitch'], self.current_pose['roll']
                )
                time.sleep(0.1)  # 调整为0.1秒，以便逐渐停止
                # 在 STOP 状态保持，不自动切换状态，等待颜色检测自动重新启动

            # 执行移动指令
            if self.status == PuppyStatus.NORMAL:
                pass  # 已在之前设置了 move_x 和 yaw_rate 并执行了 move()
            elif self.status in [PuppyStatus.ADJUSTING_YAW, PuppyStatus.ADJUSTING_X]:
                pass  # 已在各自的状态中处理了移动指令
            elif self.status == PuppyStatus.FOUND_TARGET:
                pass
            elif self.status == PuppyStatus.STOP:
                pass

            time.sleep(0.02)

            with self.status_lock:
                if self.status_last != self.status:
                    print(f'puppyStatus 由 {self.status_last} 切换为 {self.status}')
                    self.status_last = self.status

class ControlServiceNode(Node):
    def __init__(self, controller):
        super().__init__('control_service_node')
        self.controller = controller

        # 初始化 CvBridge
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.image_sub  

        self.latest_image = None
        self.image_lock = threading.Lock()

        # 初始化服务
        self.enter_service = self.create_service(Trigger, 'color_tracking/enter', self.enter_callback)
        self.enable_service = self.create_service(SetBool, 'color_tracking/enable_color_tracking', self.enable_callback)
        self.set_color_service = self.create_service(SetColor, 'color_tracking/set_color', self.set_color_callback)
        self.stop_service = self.create_service(Trigger, 'color_tracking/stop', self.stop_callback)

        self.get_logger().info('控制服务节点已启动，等待服务请求...')
    
    def image_callback(self, msg):
        """图像订阅回调函数，接收最新图像"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_image = cv_image
        except CvBridgeError as e:
            self.get_logger().error(f"转换图像失败: {e}")

    def get_latest_image(self):
        """获取最新的图像"""
        with self.image_lock:
            return self.latest_image.copy() if self.latest_image is not None else None

    # 服务回调函数
    def enter_callback(self, request, response):
        """进入颜色跟踪模式的服务回调"""
        with self.controller.status_lock:
            if self.controller.status == PuppyStatus.STOP:
                response.success = False
                response.message = "机器人处于停止状态，无法进入颜色跟踪模式。"
                self.get_logger().warn(response.message)
                return response

            if self.controller.target_color == 'None':
                response.success = False
                response.message = '未设置目标颜色，请先调用 /color_tracking/set_color 服务设置目标颜色。'
                self.get_logger().warn(response.message)
                return response

            if self.controller.status not in [PuppyStatus.START, PuppyStatus.NORMAL, PuppyStatus.FOUND_TARGET]:
                self.controller.status = PuppyStatus.START
                response.success = True
                response.message = '进入颜色跟踪模式'
                self.get_logger().info('接收到进入颜色跟踪模式的请求')
            else:
                response.success = False
                response.message = '颜色跟踪模式已经在运行'
        
        return response

    def enable_callback(self, request, response):
        """启用或禁用颜色跟踪的服务回调"""
        with self.controller.status_lock:
            if self.controller.status == PuppyStatus.STOP:
                response.success = False
                response.message = "机器人处于停止状态，无法修改颜色跟踪设置。"
                self.get_logger().warn(response.message)
                return response

            if request.data:
                if self.controller.target_color == 'None':
                    response.success = False
                    response.message = "目标颜色未设置，请先调用 /color_tracking/set_color 服务设置目标颜色。"
                    self.get_logger().warn(response.message)
                    return response
                self.controller.color_tracking_enabled = True
                self.controller.status = PuppyStatus.START  # 启动任务
                response.success = True
                response.message = "颜色跟踪已启用并启动任务"
                self.get_logger().info(response.message)
            else:
                self.controller.color_tracking_enabled = False
                self.controller.status = PuppyStatus.STOP  # 停止任务
                response.success = True
                response.message = "颜色跟踪已禁用并停止任务"
                self.get_logger().info(response.message)
        return response

    def set_color_callback(self, request, response):
        """设置目标颜色的服务回调"""
        # set_color 服务始终可用，即使在 STOP 状态下
        if request.color in self.controller.lab_data:
            with self.controller.status_lock:
                self.controller.target_color = request.color
                response.success = True
                response.message = f"目标颜色已设置为 {self.controller.target_color}"
                self.get_logger().info(response.message)
                # 设置颜色后，自动启用颜色跟踪并启动任务
                self.controller.color_tracking_enabled = True
                self.controller.status = PuppyStatus.START
                self.get_logger().info("颜色设置后，自动启动颜色跟踪任务")
        else:
            response.success = False
            response.message = f"目标颜色 {request.color} 未在配置中找到"
            self.get_logger().warn(response.message)
        return response

    def stop_callback(self, request, response):
        """停止颜色跟踪的服务回调"""
        with self.controller.status_lock:
            self.controller.color_tracking_enabled = False
            self.controller.status = PuppyStatus.STOP
            response.success = True
            response.message = "颜色跟踪已停止"
            self.get_logger().info(response.message)
        return response

def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    # 初始化控制器
    controller = PuppyController()

    # 初始化控制服务节点
    control_node = ControlServiceNode(controller)

    try:
        while rclpy.ok():
            rclpy.spin_once(control_node, timeout_sec=0.01)
            img = control_node.get_latest_image()
            if img is not None:
                frame = img.copy()
                processed_frame = controller.process_image(frame)           
                cv2.imshow('Frame', processed_frame)
                key = cv2.waitKey(1)
                if key == 27:
                    break
            else:
                time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        try:
            controller.puppy.move_stop(servo_run_time=500)
        except Exception as e:
            print(f"停止移动时出错: {e}")
        controller.puppy.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
