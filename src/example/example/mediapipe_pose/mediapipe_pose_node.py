#!/usr/bin/env python3
# encoding: utf-8
# @Author: liuyuan
# @Date: 2025/01/13

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from enum import Enum
from dataclasses import dataclass, field
import cv2
import mediapipe as mp
import time
import signal
import sys
import math
import queue
import threading
import queue
import numpy as np
from sdk import PID
from collections import deque
from puppy_control_msgs.srv import SetRunActionName
from std_srvs.srv import Trigger, Empty
from ros_robot_controller_msgs.msg import BuzzerState,PWMServoState,SetPWMServoState


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
    pid: pid = field(default_factory=lambda: pid())
    #pid: PID = field(default_factory=lambda: PID())
    current_pulse: int = field(init=False)

    def __post_init__(self):
        self.current_pulse = self.initial_pulse


class MediaPipePoseNode(Node):
    def __init__(self):
        super(MediaPipePoseNode, self).__init__('mediapipe_pose_node')

        self.scale_percent = 50
        self.alpha = 0.2  

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

        self.servos = {
            'left_upper': Servo(
                name='left_upper',
                channel=1,
                angle_min=60,
                angle_max=120,
                pulse_min=862,
                pulse_max=1966,
                initial_pulse=1011,
                invert=False,
                pid=PID.PID(P=1.0, I=0.05, D=0.01)
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
                pid=PID.PID(P=1.0, I=0.05, D=0.01)
            ),
            'right_upper': Servo(
                name='right_upper',
                channel=3,
                angle_min=60,
                angle_max=120,
                pulse_min=981,
                pulse_max=2138,
                initial_pulse=1989,
                invert=True,
                pid=PID.PID(P=1.0, I=0.05, D=0.01)
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
                pid=PID.PID(P=1.0, I=0.05, D=0.01)
            )
        }


        # 订阅摄像头图像话题
        self.image_sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)       
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.pwm_pub = self.create_publisher(SetPWMServoState,'ros_robot_controller/pwm_servo/set_state',10)
        self.run_action_group_srv = self.create_client(SetRunActionName, 'puppy_control/runActionGroup')
        self.run_action_group_srv.wait_for_service()
        self.cli = self.create_client(Empty,'puppy_control/go_home')

        self.play_mode = False
        self.stop_event = threading.Event()

        self.angle_queue = queue.Queue(maxsize=50)
        self.display_queue = queue.Queue(maxsize=10)
        
        msg = SetRunActionName.Request()
        msg.name = '2_legs_stand.d6ac'
        msg.wait = True
        self.run_action_group_srv.call_async(msg)      

        # 初始化线程
        self.servo_thread = threading.Thread(target=self.servo_control_thread, name="ServoControlThread")
        self.servo_thread.start()

        self.display_thread = threading.Thread(target=self.image_display_thread, name="ImageDisplayThread")
        self.display_thread.start()


        #for servo in self.servos.values():
            #servo.current_pulse = servo.initial_pulse

        self.angle_history = {
            'left_upper': deque(maxlen=5),
            'right_upper': deque(maxlen=5),
            'left_forearm': deque(maxlen=5),
            'right_forearm': deque(maxlen=5)
        }


        self.filtered_angles = {
            'left_upper': None,
            'right_upper': None,
            'left_forearm': None,
            'right_forearm': None
        }
        # 手部交叉检测历史
        self.hands_crossed_history = deque(maxlen=10)

        self.get_logger().info("MediaPipePoseNode 初始化完成。")

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
            self.get_logger().info("检测到的关键点数量不足以判断 T 姿势。")
            return False

        angle_threshold = 20
        horizontal_threshold = 20

        left_arm_angle = self.calculate_angle(left_shoulder, left_elbow, left_wrist)
        right_arm_angle = self.calculate_angle(right_shoulder, right_elbow, right_wrist)


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
        except IndexError:
            self.get_logger().info("检测到的关键点数量不足以判断双手交叉抱胸。")
            return False

        chest_upper = nose[1]
        chest_lower = min(left_shoulder[1], right_shoulder[1]) + 50

        left_wrist_in_chest = chest_upper < left_wrist[1] < chest_lower
        right_wrist_in_chest = chest_upper < right_wrist[1] < chest_lower


        if not (left_wrist_in_chest and right_wrist_in_chest):
            return False

        left_wrist_right_of_right_shoulder = left_wrist[0] > right_shoulder[0]
        right_wrist_left_of_left_shoulder = right_wrist[0] < left_shoulder[0]

        if not (left_wrist_right_of_right_shoulder and right_wrist_left_of_left_shoulder):
            return False

        left_elbow_angle = self.calculate_angle(left_wrist, left_elbow, left_shoulder)
        right_elbow_angle = self.calculate_angle(right_wrist, right_elbow, right_shoulder)

        elbow_angle_threshold = 70
        elbows_bent = left_elbow_angle < elbow_angle_threshold and right_elbow_angle < elbow_angle_threshold

        hands_distance = abs(left_wrist[0] - right_wrist[0])
        hands_distance_threshold = 100  # 像素


        hands_close = hands_distance < hands_distance_threshold

        return elbows_bent and hands_close

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
            self.get_logger().info(f"计算角度时出错: {e}")
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
        return self.filtered_angles[servo_name]

    def map_arm_angle_to_pulse(self, angle, servo: Servo):
        """
        将手臂角度映射到伺服舵机的脉冲宽度
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

        return pulse
        
    def map_arm_angle_to_pulse01(self, angle, servo: Servo):
        """
        将手臂角度映射到伺服舵机的脉冲宽度
        """
        # 限制角度在伺服的范围内
        clamped_angle = max(servo.angle_min, min(servo.angle_max, angle))

        if servo.invert:
            clamped_angle = servo.angle_max - (clamped_angle - servo.angle_min)
            

        # 线性映射角度到脉冲宽度
        pulse = ((servo.angle_max - clamped_angle) / (servo.angle_max - servo.angle_min)) * \
                (servo.pulse_max - servo.pulse_min) + servo.pulse_min
        pulse = int(round(pulse))

        # 限制脉冲在有效范围内
        pulse = max(servo.pulse_min, min(servo.pulse_max, pulse))

        return pulse

    def control_servo_based_on_pulses(self, target_pulses):
        """
        平滑地将当前脉冲宽度过渡到目标脉冲宽度
        """
        step = 10  #
        msg = SetPWMServoState()
        msg.duration = 0.2

        adjustments = True
        while adjustments and not self.stop_event.is_set():
            adjustments = False
            pwm_list = []
            for servo_name, target_pulse in target_pulses.items():
                servo = self.servos[servo_name]
                current_pulse = servo.current_pulse
                if abs(current_pulse - target_pulse) > step:
                    adjustments = True
                    if current_pulse < target_pulse:
                        servo.current_pulse += step
                    else:
                        servo.current_pulse -= step
                    pos = PWMServoState()
                    pos.id = [servo.channel]
                    pos.position = [int(servo.current_pulse)]
                    pwm_list.append(pos)
            msg.state = pwm_list
            self.pwm_pub.publish(msg)

    def servo_control_thread(self):
        while not self.stop_event.is_set():
            try:
                pulses = self.angle_queue.get(timeout=0.1)
                self.control_servo_based_on_pulses(pulses)
            except queue.Empty:
                continue
            except Exception as e:
                pass

    def image_display_thread(self):
        """图像显示线程"""
        self.get_logger().info("图像显示线程启动。")
        while not self.stop_event.is_set():
            try:
                cv_image = self.display_queue.get(timeout=0.1)
                cv2.imshow("MediaPipe Pose", cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info("按下 'q' 键，关闭窗口并退出。")
                    self.stop_all()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().info(f"图像显示线程出错: {e}")
        cv2.destroyAllWindows()
        self.get_logger().info("图像显示线程停止。")

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将 ROS 图像消息转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (640, 480), interpolation=cv2.INTER_AREA)

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
                    self.get_logger().info("图像显示队列已满，丢弃当前帧。")
        except Exception as e:
            self.get_logger().info(f"处理图像时出错: {e}")

    def buzzer_warn(self):
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 2
        self.buzzer_pub.publish(msg) 
    
    def enter_play_mode(self):
        """进入玩法模式"""
        self.play_mode = True

        # 定义期望角度
        # 根据实际需求调整这些值
        desired_set_point_left_upper = 90     
        desired_set_point_right_upper = 90    
        desired_set_point_left_forearm = 135   
        desired_set_point_right_forearm = 135 

        # 设置 PID 控制器的目标点
        self.servos['left_upper'].pid.SetPoint = desired_set_point_left_upper
        self.servos['right_upper'].pid.SetPoint = desired_set_point_right_upper
        self.servos['left_forearm'].pid.SetPoint = desired_set_point_left_forearm
        self.servos['right_forearm'].pid.SetPoint = desired_set_point_right_forearm

        

        # 播放蜂鸣器声音
        self.buzzer_warn()

    def exit_play_mode(self):
        """退出玩法模式"""
        self.play_mode = False
        self.get_logger().info("Exiting play mode.")


        # 播放退出蜂鸣器声音
        self.buzzer_warn()

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
            pulse_correction_left_upper = self.servos['left_upper'].pid.output
            pulse_left_upper = self.servos['left_upper'].current_pulse + pulse_correction_left_upper
            pulse_left_upper = self.map_arm_angle_to_pulse01(self.filtered_angles['left_upper'], self.servos['left_upper'])
            # 计算右上臂角度
            right_shoulder_angle = self.calculate_shoulder_angle(right_shoulder, right_elbow, nose)
            self.filtered_angles['right_upper'] = self.apply_low_pass_filter('right_upper', right_shoulder_angle)
            self.servos['right_upper'].pid.update(right_shoulder_angle)
            pulse_correction_right_upper = self.servos['right_upper'].pid.output
            pulse_right_upper = self.servos['right_upper'].current_pulse + pulse_correction_right_upper
            pulse_right_upper = self.map_arm_angle_to_pulse01(self.filtered_angles['right_upper'], self.servos['right_upper'])
            # 计算左前臂角度
            left_forearm_angle = self.calculate_angle(left_shoulder, left_elbow, left_wrist)
            self.filtered_angles['left_forearm'] = self.apply_low_pass_filter('left_forearm', left_forearm_angle)
            self.servos['left_forearm'].pid.update(left_forearm_angle)
            pulse_correction_left_forearm = self.servos['left_forearm'].pid.output
            pulse_left_forearm = self.servos['left_forearm'].current_pulse + pulse_correction_left_forearm
            pulse_left_forearm = self.map_arm_angle_to_pulse(self.filtered_angles['left_forearm'], self.servos['left_forearm'])
            # 计算右前臂角度
            right_forearm_angle = self.calculate_angle(right_shoulder, right_elbow, right_wrist)
            self.filtered_angles['right_forearm'] = self.apply_low_pass_filter('right_forearm', right_forearm_angle)
            self.servos['right_forearm'].pid.update(right_forearm_angle)
            pulse_correction_right_forearm = self.servos['right_forearm'].pid.output
            pulse_right_forearm = self.servos['right_forearm'].current_pulse + pulse_correction_right_forearm
            pulse_right_forearm = self.map_arm_angle_to_pulse(self.filtered_angles['right_forearm'], self.servos['right_forearm'])

            pulses = {
                'left_upper': pulse_left_upper,
                'right_upper': pulse_right_upper,
                'left_forearm': pulse_left_forearm,
                'right_forearm': pulse_right_forearm
            }
            self.angle_queue.put(pulses)

            if self.is_hands_crossed_on_chest(pixel_landmarks):
                self.exit_play_mode()
        except IndexError:
            self.get_logger().info("关键点检测不到，无法处理玩法模式。")
        except Exception as e:
            self.get_logger().info(f"处理玩法模式时出错: {e}")

    def run(self):
        """运行节点"""
        rclpy.spin(self)

    def stop_all(self):
        """关闭节点"""
        self.cli.call_async(Empty.Request())
        rclpy.shutdown()
        time.sleep(2)
        if not self.stop_event.is_set():
            self.get_logger().info("正在关闭 MediaPipePoseNode...")
            self.stop_event.set()
            self.servo_thread.join()
            self.display_thread.join()
            cv2.destroyAllWindows()
            self.get_logger().info("MediaPipePoseNode 已关闭。")

    def signal_handler(self, signum, frame):
        """信号处理器，实现关闭"""
        self.get_logger().info("接收到退出信号，正在关闭...")
        self.stop_all()
        sys.exit(0)
        
def main():
    rclpy.init()
    node = MediaPipePoseNode()
    try:
        signal.signal(signal.SIGINT, node.signal_handler)
        signal.signal(signal.SIGTERM, node.signal_handler)
        node.run()
    except KeyboardInterrupt:
        node.stop_all()

if __name__ == '__main__':
    main()
