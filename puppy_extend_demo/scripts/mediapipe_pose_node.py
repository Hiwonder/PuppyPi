#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
MediaPipePoseNode ROS 

MediaPipe 进行姿态检测，并根据检测结果控制四个舵机。每个舵机都配备了一个 PID 控制器
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time
import signal
import sys
import math
import threading
import queue
from collections import deque
from enum import Enum
from dataclasses import dataclass, field

sys.path.append('/home/ubuntu/software/puppypi_control')

from puppy_control.msg import Velocity, Pose, Gait, SetServo
from ros_robot_controller_sdk import Board
from pwm_servo_control import PWMServoControl
from ros_robot_controller.msg import BuzzerState 
from puppy_control.srv import SetRunActionName
from action_group_control import runActionGroup, stopActionGroup
from std_srvs.srv import Empty  


class Landmark(Enum):
    NOSE = 0
    LEFT_SHOULDER = 11
    RIGHT_SHOULDER = 12
    LEFT_ELBOW = 13
    RIGHT_ELBOW = 14
    LEFT_WRIST = 15
    RIGHT_WRIST = 16
    LEFT_HIP = 23
    RIGHT_HIP = 24


class PID:
    """PID Controller"""

    def __init__(self, P=1.0, I=0.05, D=0.01):
        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculates PID value for given reference feedback"""
        error = self.SetPoint - feedback_value

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if delta_time >= self.sample_time:
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if self.ITerm < -self.windup_guard:
                self.ITerm = -self.windup_guard
            elif self.ITerm > self.windup_guard:
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Sets the windup guard"""
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """Sets the sample time"""
        self.sample_time = sample_time


@dataclass
class Servo:
    name: str
    channel: int
    angle_min: int
    angle_max: int
    pulse_min: int
    pulse_max: int
    initial_pulse: int
    invert: bool = False
    pid: PID = field(default_factory=lambda: PID())
    current_pulse: int = field(init=False)
    pulse_queue: queue.Queue = field(default_factory=lambda: queue.Queue(maxsize=10))

    def __post_init__(self):
        self.current_pulse = self.initial_pulse


class MediaPipePoseNode:
    """MediaPipePoseNode ROS 节点"""

    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('mediapipe_pose_node', anonymous=True)
        runActionGroup('demo.d6ac', True)
        # 读取参数
        self.scale_percent = rospy.get_param('~scale_percent', 50)  
        self.alpha = rospy.get_param('~alpha', 0.2)  

        # 初始化 CvBridge
        self.bridge = CvBridge()

        # 初始化 MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

        # 初始化舵机控制器
        self.board = Board()
        self.servo_controller = PWMServoControl(self.board)


        self.servos = {
            'left_upper': Servo(
                name='left_upper',
                channel=1,
                angle_min=60,
                angle_max=120,
                pulse_min=862,
                pulse_max=1966,
                initial_pulse=1011,
                invert=True,
                pid=PID(P=1.0, I=0.05, D=0.01)
            ),
            'left_forearm': Servo(
                name='left_forearm',
                channel=2,
                angle_min=90,
                angle_max=180,
                pulse_min=953,
                pulse_max=1818,
                initial_pulse=862,
                invert=False,
                pid=PID(P=1.0, I=0.05, D=0.01)
            ),
            'right_upper': Servo(
                name='right_upper',
                channel=3,
                angle_min=60,
                angle_max=120,
                pulse_min=981,
                pulse_max=2138,
                initial_pulse=1989,
                invert=False,
                pid=PID(P=1.0, I=0.05, D=0.01)
            ),
            'right_forearm': Servo(
                name='right_forearm',
                channel=4,
                angle_min=90,
                angle_max=180,
                pulse_min=1253,
                pulse_max=2288,
                initial_pulse=1989,
                invert=True,
                pid=PID(P=1.0, I=0.05, D=0.01)
            )
        }

        # 初始化舵机脉冲宽度
        for servo in self.servos.values():
            self.servo_controller.setPulse(servo.channel, servo.initial_pulse, 350)

        # 初始化发布者和服务
        self.buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=10)
        self.run_action_group_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
        self.PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
        self.PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
        self.PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

        # 订阅摄像头图像话题
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

        self.play_mode = False
        self.stop_event = threading.Event()

        self.display_queue = queue.Queue(maxsize=10)

        # 初始化线程
        self.servo_threads = []
        for servo in self.servos.values():
            t = threading.Thread(target=self.servo_control_thread, args=(servo,), name=f"{servo.name}_ControlThread")
            t.start()
            self.servo_threads.append(t)

        self.display_thread = threading.Thread(target=self.image_display_thread, name="ImageDisplayThread")
        self.display_thread.start()

        # 初始化当前脉冲宽度
        for servo in self.servos.values():
            servo.current_pulse = servo.initial_pulse

        self.angle_history = {
            'left_upper': deque(maxlen=5),
            'right_upper': deque(maxlen=5),
            'left_forearm': deque(maxlen=5),
            'right_forearm': deque(maxlen=5)
        }

        # 过滤后的角度
        self.filtered_angles = {
            'left_upper': None,
            'right_upper': None,
            'left_forearm': None,
            'right_forearm': None
        }

        # 手部交叉检测历史
        self.hands_crossed_history = deque(maxlen=10)  

        rospy.loginfo("MediaPipePoseNode 初始化完成。")

    def convert_landmarks_to_pixels(self, img, landmarks):
        """将 MediaPipe 关键点转换为像素坐标"""
        img_height, img_width, _ = img.shape
        return [(int(lm.x * img_width), int(lm.y * img_height)) for lm in landmarks]

    def is_T_pose(self, pixel_landmarks):
        """判断是否为 T 姿势"""
        try:
            left_shoulder = pixel_landmarks[Landmark.LEFT_SHOULDER.value]
            right_shoulder = pixel_landmarks[Landmark.RIGHT_SHOULDER.value]
            left_elbow = pixel_landmarks[Landmark.LEFT_ELBOW.value]
            right_elbow = pixel_landmarks[Landmark.RIGHT_ELBOW.value]
            left_wrist = pixel_landmarks[Landmark.LEFT_WRIST.value]
            right_wrist = pixel_landmarks[Landmark.RIGHT_WRIST.value]
        except IndexError:
            rospy.logwarn("检测到的关键点数量不足以判断 T 姿势。")
            return False

        angle_threshold = 20
        horizontal_threshold = 20

        left_arm_angle = self.calculate_angle(left_shoulder, left_elbow, left_wrist)
        right_arm_angle = self.calculate_angle(right_shoulder, right_elbow, right_wrist)

        rospy.logdebug(f"左前臂角度: {left_arm_angle:.2f} 度")
        rospy.logdebug(f"右前臂角度: {right_arm_angle:.2f} 度")

        left_straight = abs(left_arm_angle - 180) < angle_threshold
        right_straight = abs(right_arm_angle - 180) < angle_threshold

        left_horizontal = self.is_horizontal(left_shoulder, left_wrist, horizontal_threshold)
        right_horizontal = self.is_horizontal(right_shoulder, right_wrist, horizontal_threshold)

        return left_straight and right_straight and left_horizontal and right_horizontal

    def is_hands_crossed_on_chest(self, pixel_landmarks):
        """判断双手是否交叉抱胸"""
        try:
            nose = pixel_landmarks[Landmark.NOSE.value]
            left_shoulder = pixel_landmarks[Landmark.LEFT_SHOULDER.value]
            right_shoulder = pixel_landmarks[Landmark.RIGHT_SHOULDER.value]
            left_elbow = pixel_landmarks[Landmark.LEFT_ELBOW.value]
            right_elbow = pixel_landmarks[Landmark.RIGHT_ELBOW.value]
            left_wrist = pixel_landmarks[Landmark.LEFT_WRIST.value]
            right_wrist = pixel_landmarks[Landmark.RIGHT_WRIST.value]
            left_hip = pixel_landmarks[Landmark.LEFT_HIP.value]
            right_hip = pixel_landmarks[Landmark.RIGHT_HIP.value]
        except IndexError:
            rospy.logwarn("检测到的关键点数量不足以判断双手交叉抱胸。")
            return False

        # 定义胸部区域
        chest_upper = nose[1]
        chest_lower = min(left_shoulder[1], right_shoulder[1]) + 50

        # 检查手腕是否位于胸部区域
        left_wrist_in_chest = chest_upper < left_wrist[1] < chest_lower
        right_wrist_in_chest = chest_upper < right_wrist[1] < chest_lower

        rospy.logdebug(f"左手腕位于胸部区域: {left_wrist_in_chest}")
        rospy.logdebug(f"右手腕位于胸部区域: {right_wrist_in_chest}")

        if not (left_wrist_in_chest and right_wrist_in_chest):
            self.hands_crossed_history.append(False)
            return False

        # 检查手腕相对于肩膀的位置
        left_wrist_right_of_right_shoulder = left_wrist[0] > right_shoulder[0]
        right_wrist_left_of_left_shoulder = right_wrist[0] < left_shoulder[0]

        rospy.logdebug(f"左手腕位于右肩膀的右侧: {left_wrist_right_of_right_shoulder}")
        rospy.logdebug(f"右手腕位于左肩膀的左侧: {right_wrist_left_of_left_shoulder}")

        if not (left_wrist_right_of_right_shoulder and right_wrist_left_of_left_shoulder):
            self.hands_crossed_history.append(False)
            return False

        # 计算肘部角度
        left_elbow_angle = self.calculate_angle(left_wrist, left_elbow, left_shoulder)
        right_elbow_angle = self.calculate_angle(right_wrist, right_elbow, right_shoulder)

        elbow_angle_threshold = 50
        elbows_bent = left_elbow_angle < elbow_angle_threshold and right_elbow_angle < elbow_angle_threshold

        rospy.logdebug(f"左肘弯曲角度: {left_elbow_angle:.2f} 度")
        rospy.logdebug(f"右肘弯曲角度: {right_elbow_angle:.2f} 度")

        if not elbows_bent:
            self.hands_crossed_history.append(False)
            return False

        hands_distance = abs(left_wrist[0] - right_wrist[0])
        hands_distance_threshold = 100  # 像素

        rospy.logdebug(f"双手水平距离: {hands_distance} 像素")
        rospy.logdebug(f"双手水平距离阈值: {hands_distance_threshold} 像素")

        hands_close = hands_distance < hands_distance_threshold

        if hands_close and elbows_bent and left_wrist_right_of_right_shoulder and right_wrist_left_of_left_shoulder and left_wrist_in_chest and right_wrist_in_chest:
            self.hands_crossed_history.append(True)
            # 检查历史记录，确保连续多帧检测到双手交叉
            if self.hands_crossed_history.count(True) >= 5:
                return True
        else:
            self.hands_crossed_history.append(False)

        return False

    def is_horizontal(self, shoulder, wrist, threshold):
        """判断手腕是否水平"""
        vertical_diff = abs(shoulder[1] - wrist[1])
        return vertical_diff < threshold

    @staticmethod
    def calculate_angle(a, b, c):
        """
        计算点 b 处的夹角，点 a, b, c 坐标为 (x, y)
        """
        try:
            vector1 = (a[0] - b[0], a[1] - b[1])
            vector2 = (c[0] - b[0], c[1] - b[1])

            len1 = math.hypot(*vector1)
            len2 = math.hypot(*vector2)

            if len1 == 0 or len2 == 0:
                return 0.0

            dot = vector1[0] * vector2[0] + vector1[1] * vector2[1]
            angle = math.acos(max(min(dot / (len1 * len2), 1.0), -1.0)) * (180.0 / math.pi)
            return angle
        except Exception as e:
            rospy.logwarn(f"计算角度时出错: {e}")
            return 0.0

    def calculate_shoulder_angle(self, shoulder, elbow, neck):
        """计算肩部角度"""
        return self.calculate_angle(elbow, shoulder, neck)

    def apply_low_pass_filter(self, servo_name, new_angle):
        """应用低通滤波器平滑角度"""
        if self.filtered_angles[servo_name] is None:
            self.filtered_angles[servo_name] = new_angle
        else:
            self.filtered_angles[servo_name] = self.alpha * new_angle + (1 - self.alpha) * self.filtered_angles[servo_name]
        rospy.logdebug(f"{servo_name} 滤波后的角度: {self.filtered_angles[servo_name]:.2f} 度")
        return self.filtered_angles[servo_name]

    def map_arm_angle_to_pulse(self, angle, servo: Servo):
        """
        将角度映射到脉冲宽度
        """
        # 限制角度在伺服的范围内
        clamped_angle = max(servo.angle_min, min(servo.angle_max, angle))

        if servo.invert:
            clamped_angle = servo.angle_max - (clamped_angle - servo.angle_min)

        # 线性映射角度到脉冲宽度
        pulse = ((clamped_angle - servo.angle_min) / (servo.angle_max - servo.angle_min)) * \
                (servo.pulse_max - servo.pulse_min) + servo.pulse_min
        pulse = int(round(pulse))

        # 限制脉冲在有效范围内
        pulse = max(servo.pulse_min, min(servo.pulse_max, pulse))

        rospy.logdebug(f"将角度 {angle:.2f}° 映射到脉冲 {pulse}μs（伺服通道 {servo.channel}）")
        return pulse

    def servo_control_thread(self, servo: Servo):
        """独立的舵机控制线程"""
        rospy.loginfo(f"{servo.name} 控制线程启动。")
        while not self.stop_event.is_set():
            try:
                target_pulse = servo.pulse_queue.get(timeout=0.1)
                self.smooth_pulse_transition(servo, target_pulse)
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"{servo.name} 控制线程出错: {e}")
        rospy.loginfo(f"{servo.name} 控制线程停止。")

    def smooth_pulse_transition(self, servo: Servo, target_pulse: int):
        """
        平滑地将当前脉冲宽度过渡到目标脉冲宽度
        """
        step = 10  # 增大步进值
        delay = 0.001  # 减少延迟时间（秒）

        while abs(servo.current_pulse - target_pulse) > step and not self.stop_event.is_set():
            if servo.current_pulse < target_pulse:
                servo.current_pulse += step
            else:
                servo.current_pulse -= step
            # 确保脉冲宽度不超出范围
            servo.current_pulse = max(servo.pulse_min, min(servo.pulse_max, servo.current_pulse))
            self.servo_controller.setPulse(servo.channel, servo.current_pulse, int(delay * 1000))
            rospy.logdebug(f"{servo.name} 调整到脉冲 {servo.current_pulse}μs")
            time.sleep(delay)

        # 设置最终脉冲宽度
        servo.current_pulse = target_pulse
        self.servo_controller.setPulse(servo.channel, servo.current_pulse, int(delay * 1000))
        rospy.logdebug(f"{servo.name} 最终设置到脉冲 {servo.current_pulse}μs")

    def image_display_thread(self):
        """图像显示线程"""
        rospy.loginfo("图像显示线程启动。")
        while not self.stop_event.is_set():
            try:
                cv_image = self.display_queue.get(timeout=0.1)
                cv2.imshow("MediaPipe Pose", cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo("按下 'q' 键，关闭窗口并退出。")
                    self.stop_all()
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"图像显示线程出错: {e}")
        cv2.destroyAllWindows()
        rospy.loginfo("图像显示线程停止。")

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            width = int(cv_image.shape[1] * self.scale_percent / 100)
            height = int(cv_image.shape[0] * self.scale_percent / 100)
            cv_image = cv2.resize(cv_image, (width, height), interpolation=cv2.INTER_AREA)

            # 转换为 RGB 并使用 MediaPipe 处理
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = self.pose.process(rgb_image)

            if results.pose_landmarks:
                self.mp_drawing.draw_landmarks(cv_image, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)
                pixel_landmarks = self.convert_landmarks_to_pixels(cv_image, results.pose_landmarks.landmark)

                if not self.play_mode:
                    if self.is_T_pose(pixel_landmarks):
                        self.enter_play_mode()
                else:
                    self.process_play_mode(pixel_landmarks)
                try:
                    self.display_queue.put_nowait(cv_image)
                except queue.Full:
                    rospy.logwarn("图像显示队列已满，丢弃当前帧。")
        except Exception as e:
            rospy.logerr(f"处理图像时出错: {e}")

    def enter_play_mode(self):
        """进入玩法模式"""
        self.play_mode = True
        rospy.loginfo("T-pose detected. Entering play mode.")
        rospy.logdebug("T-pose detected. Entering play mode.")

        desired_set_point = {
            'left_upper': 90,     
            'right_upper': 90,    
            'left_forearm': 135,   
            'right_forearm': 135 
        }

        # 设置 PID 控制器的目标点
        for servo_name, set_point in desired_set_point.items():
            self.servos[servo_name].pid.SetPoint = set_point

        buzzer_msg = BuzzerState(freq=2400, on_time=0.1, off_time=0.9, repeat=1)
        self.buzzer_pub.publish(buzzer_msg)

    def exit_play_mode(self):
        """退出玩法模式"""
        self.play_mode = False
        rospy.loginfo("Exiting play mode.")
        rospy.logdebug("Exiting play mode.")

        # 执行动作组
        try:
            runActionGroup('stand.d6a', True)
            rospy.loginfo("已执行退出玩法模式的动作组 'stand.d6a'。")
        except Exception as e:
            rospy.logerr(f"执行退出玩法模式动作组时出错: {e}")

        buzzer_msg_exit = BuzzerState(freq=2000, on_time=0.2, off_time=0.8, repeat=1)
        self.buzzer_pub.publish(buzzer_msg_exit)

    def process_play_mode(self, pixel_landmarks):
        """处理玩法模式下的舵机控制"""
        try:
            # 提取相关关键点
            nose = pixel_landmarks[Landmark.NOSE.value]
            left_shoulder = pixel_landmarks[Landmark.LEFT_SHOULDER.value]
            right_shoulder = pixel_landmarks[Landmark.RIGHT_SHOULDER.value]
            left_elbow = pixel_landmarks[Landmark.LEFT_ELBOW.value]
            right_elbow = pixel_landmarks[Landmark.RIGHT_ELBOW.value]
            left_wrist = pixel_landmarks[Landmark.LEFT_WRIST.value]
            right_wrist = pixel_landmarks[Landmark.RIGHT_WRIST.value]
            left_hip = pixel_landmarks[Landmark.LEFT_HIP.value]
            right_hip = pixel_landmarks[Landmark.RIGHT_HIP.value]

            # 计算左上臂角度
            left_shoulder_angle = self.calculate_shoulder_angle(left_shoulder, left_elbow, nose)
            self.filtered_angles['left_upper'] = self.apply_low_pass_filter('left_upper', left_shoulder_angle)
            self.servos['left_upper'].pid.update(left_shoulder_angle)
            pulse_left_upper = self.map_arm_angle_to_pulse(self.filtered_angles['left_upper'], self.servos['left_upper'])

            # 计算右上臂角度
            right_shoulder_angle = self.calculate_shoulder_angle(right_shoulder, right_elbow, nose)
            self.filtered_angles['right_upper'] = self.apply_low_pass_filter('right_upper', right_shoulder_angle)
            self.servos['right_upper'].pid.update(right_shoulder_angle)
            pulse_right_upper = self.map_arm_angle_to_pulse(self.filtered_angles['right_upper'], self.servos['right_upper'])

            # 计算左前臂角度
            left_forearm_angle = self.calculate_angle(left_shoulder, left_elbow, left_wrist)
            self.filtered_angles['left_forearm'] = self.apply_low_pass_filter('left_forearm', left_forearm_angle)
            self.servos['left_forearm'].pid.update(left_forearm_angle)
            pulse_left_forearm = self.map_arm_angle_to_pulse(self.filtered_angles['left_forearm'], self.servos['left_forearm'])

            # 计算右前臂角度
            right_forearm_angle = self.calculate_angle(right_shoulder, right_elbow, right_wrist)
            self.filtered_angles['right_forearm'] = self.apply_low_pass_filter('right_forearm', right_forearm_angle)
            self.servos['right_forearm'].pid.update(right_forearm_angle)
            pulse_right_forearm = self.map_arm_angle_to_pulse(self.filtered_angles['right_forearm'], self.servos['right_forearm'])

            pulses = {
                'left_upper': pulse_left_upper,
                'right_upper': pulse_right_upper,
                'left_forearm': pulse_left_forearm,
                'right_forearm': pulse_right_forearm
            }

            # 将脉冲分别放入各自的队列
            for servo_name, pulse in pulses.items():
                try:
                    self.servos[servo_name].pulse_queue.put_nowait(pulse)
                except queue.Full:
                    rospy.logwarn(f"{servo_name} 脉冲队列已满，丢弃当前脉冲。")

            if self.is_hands_crossed_on_chest(pixel_landmarks):
                self.exit_play_mode()
        except IndexError:
            rospy.logwarn("关键点检测不到，无法处理玩法模式。")
        except Exception as e:
            rospy.logerr(f"处理玩法模式时出错: {e}")

    def run(self):
        """运行节点"""
        rospy.on_shutdown(self.stop_all)
        rospy.spin()

    def stop_all(self):
        """优雅关闭节点"""
        if not self.stop_event.is_set():
            rospy.loginfo("正在关闭 MediaPipePoseNode...")
            self.stop_event.set()
            # 等待所有舵机控制线程结束
            for t in self.servo_threads:
                t.join()
            # 等待图像显示线程结束
            self.display_thread.join()
            cv2.destroyAllWindows()
            rospy.loginfo("MediaPipePoseNode 已关闭。")

    def signal_handler(self, signum, frame):
        rospy.loginfo("接收到退出信号，正在优雅关闭...")
        self.stop_all()
        sys.exit(0)


if __name__ == '__main__':
    try:
        node = MediaPipePoseNode()

        signal.signal(signal.SIGINT, node.signal_handler)
        signal.signal(signal.SIGTERM, node.signal_handler)

        node.run()
    except rospy.ROSInterruptException:
        pass
