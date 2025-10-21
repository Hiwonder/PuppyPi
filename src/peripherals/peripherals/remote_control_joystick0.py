#!/usr/bin/env python3
# coding=utf8
import os
import sys
import rclpy
from rclpy.node import Node
import threading
import math
import time
import numpy as np
import pygame as pg
from sensor_msgs.msg import Joy
from functools import partial
from ros_robot_controller_msgs.msg import BuzzerState
from geometry_msgs.msg import Polygon
from puppy_control_msgs.msg import Velocity, Pose
from std_srvs.srv import Empty

# PuppyPose
PuppyMove = {'x': 0.0, 'y': 0.0, 'yaw_rate': 0.0}  # 确保所有初始值为 float 类型
Stand = {'roll': math.radians(0), 'pitch': math.radians(0), 'yaw': 0.0, 'height': -10.0, 'x_shift': -0.5, 'stance_x': 0.0, 'stance_y': 0.0}  # 确保所有初始值为 float 类型

ROS_NODE_NAME = 'remote_control_joystick'

enable_control = True

# 按键动作映射
PRESSED_ACTION_MAP = {}
HOLD_ACTION_MAP = {}
RELEASED_ACTION_MAP = {}

BUTTONS = ("CROSS", "CIRCLE", "None_1", "SQUARE",
           "TRIANGLE", "None_2", "L1", "R1",
           "L2", "R2", "SELECT", "START", "MODE",
           "L_HAT_LEFT", "L_HAT_RIGHT", "L_HAT_DOWN", "L_HAT_UP",
           "L_AXIS_LEFT", "L_AXIS_RIGHT", "L_AXIS_UP", "L_AXIS_DOWN",
           "R_AXIS_LEFT", "R_AXIS_RIGHT", "R_AXIS_UP", "R_AXIS_DOWN")

LegsCoord = None
VelocityX = 15.0  # 确保初始值为 float 类型

class RemoteControlNode(Node):
    def __init__(self):
        super().__init__(ROS_NODE_NAME)
        global PuppyPose, PuppyMove
        PuppyPose = Stand.copy()
        PuppyMove = {'x': 0.0, 'y': 0.0, 'yaw_rate': 0.0}  # 确保初始值为 float 类型

        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.PuppyVelocityPub = self.create_publisher(Velocity, '/puppy_control/velocity/autogait', 1)
        self.PuppyPosePub = self.create_publisher(Pose, '/puppy_control/pose', 1)

        self.legs_coord_sub = self.create_subscription(Polygon, '/puppy_control/legs_coord', self.LegsCoordFun, 1)

        self.create_timer(0.05, self.update_buttons)

        self.js = Joystick()
        self.init_button_actions()

    def init_button_actions(self):
        global PRESSED_ACTION_MAP, HOLD_ACTION_MAP, RELEASED_ACTION_MAP
        # 定义按键动作
        PRESSED_ACTION_MAP = {
            "CROSS": partial(self.PressedFun, 1.0, key='x'),  # 前进
            "CIRCLE": partial(self.PressedFun, 1.0, key='yaw_rate'),  # 右转
            "SQUARE": partial(self.PressedFun, -1.0, key='yaw_rate'),  # 左转
            "TRIANGLE": partial(self.PressedFun, -1.0, key='x'),  # 后退
            "L_HAT_UP": partial(self.PressedFun, 1.0, key='height'),  # 抬高
            "L_HAT_DOWN": partial(self.PressedFun, -1.0, key='height'),  # 降低
            "L_HAT_LEFT": partial(self.PressedFun, -1.0, key='x_shift'),  # 左移
            "L_HAT_RIGHT": partial(self.PressedFun, 1.0, key='x_shift'),  # 右移
        }

        HOLD_ACTION_MAP = {
            "CIRCLE": partial(self.HoldFun, 1.0, key='yaw_rate'),  # 右转
            "SQUARE": partial(self.HoldFun, -1.0, key='yaw_rate'),  # 左转
            "L_HAT_UP": partial(self.HoldFun, 1.0, key='height'),  # 抬高
            "L_HAT_DOWN": partial(self.HoldFun, -1.0, key='height'),  # 降低
        }

        RELEASED_ACTION_MAP = {
            "CROSS": partial(self.ReleaseFun, 0.0, key='x'),  # 停止前进
            "CIRCLE": partial(self.ReleaseFun, 0.0, key='yaw_rate'),  # 停止右转
            "SQUARE": partial(self.ReleaseFun, 0.0, key='yaw_rate'),  # 停止左转
            "TRIANGLE": partial(self.ReleaseFun, 0.0, key='x'),  # 停止后退
        }

    def go_home(self):
        global PuppyPose, PuppyMove
        self.get_logger().info('go_home')
        PuppyPose = Stand.copy()
        PuppyMove = {'x': 0.0, 'y': 0.0, 'yaw_rate': 0.0}  # 确保初始值为 float 类型
        self.call_service('/puppy_control/go_home')
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.1
        msg.off_time = 0.9
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
        self.get_logger().info(str(PuppyPose))

    def call_service(self, service_name):
        client = self.create_client(Empty, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} service not available, waiting again...')
        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def LegsCoordFun(self, msg):
        global LegsCoord
        LegsCoord = msg

    def PressedFun(self, *args, **kwargs):
        global LegsCoord
        global VelocityX

        if (self.js.get_button_state('L_HAT_UP') or self.js.get_button_state('L_AXIS_UP')
                or self.js.get_button_state('L_HAT_DOWN') or self.js.get_button_state('L_AXIS_DOWN')):
            if kwargs['key'] == 'CIRCLE':
                VelocityX += 1.0  # 确保类型为 float
            elif kwargs['key'] == 'SQUARE':
                VelocityX -= 1.0  # 确保类型为 float

            if VelocityX > 25.0:
                VelocityX = 25.0
            elif VelocityX < 5.0:
                VelocityX = 5.0

            if PuppyMove['x'] > 0.0:
                PuppyMove['x'] = VelocityX
            elif PuppyMove['x'] < 0.0:
                PuppyMove['x'] = -VelocityX
            self.PuppyVelocityPub.publish(Velocity(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate']))
            self.get_logger().info(str(PuppyMove['x']))

        if kwargs['key'] in PuppyMove.keys():
            if kwargs['key'] == 'x':
                if args[0] > 0.0:
                    PuppyMove['x'] = float(VelocityX)  # 确保 x 是 float 类型
                else:
                    PuppyMove['x'] = float(-VelocityX)  # 确保 x 是 float 类型
            else:
                PuppyMove[kwargs['key']] = float(args[0])  # 确保所有字段都是 float 类型
            self.PuppyVelocityPub.publish(Velocity(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate']))
            self.get_logger().info(str(PuppyMove['x']))

        elif kwargs['key'] in PuppyPose.keys():
            if kwargs['key'] == 'height':
                PuppyPose['height'] += float(args[0])
                if PuppyPose['height'] < -16.0:
                    PuppyPose['height'] = -16.0
                if PuppyPose['height'] > -5.0:
                    PuppyPose['height'] = -5.0
            if kwargs['key'] == 'x_shift':
                PuppyPose['x_shift'] += float(args[0])
                if PuppyPose['x_shift'] < -6.0:
                    PuppyPose['x_shift'] = -6.0
                if PuppyPose['x_shift'] > 6.0:
                    PuppyPose['x_shift'] = 6.0
            if kwargs['key'] == 'pitch':
                PuppyPose['pitch'] += float(args[0])
                if PuppyPose['pitch'] < -np.radians(20.0):
                    PuppyPose['pitch'] = -np.radians(20.0)
                if PuppyPose['pitch'] > np.radians(20.0):
                    PuppyPose['pitch'] = np.radians(20.0)
            self.PuppyPosePub.publish(Pose(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'],
                                           x_shift=PuppyPose['x_shift'], height=PuppyPose['height'],
                                           roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw']))
            self.get_logger().info(str(PuppyPose))
        elif kwargs['key'] == 'legs_coord':
            print(LegsCoord)

    def HoldFun(self, *args, **kwargs):
        global VelocityX

        if kwargs['key'] in PuppyPose.keys():
            if kwargs['key'] == 'height':
                PuppyPose['height'] += float(args[0])
                if PuppyPose['height'] < -16.0:
                    PuppyPose['height'] = -16.0
                if PuppyPose['height'] > -5.0:
                    PuppyPose['height'] = -5.0
            if kwargs['key'] == 'x_shift':
                PuppyPose['x_shift'] += float(args[0])
                if PuppyPose['x_shift'] < -6.0:
                    PuppyPose['x_shift'] = -6.0
                if PuppyPose['x_shift'] > 6.0:
                    PuppyPose['x_shift'] = 6.0
            if kwargs['key'] == 'pitch':
                PuppyPose['pitch'] += float(args[0])
                if PuppyPose['pitch'] < -np.radians(20.0):
                    PuppyPose['pitch'] = -np.radians(20.0)
                if PuppyPose['pitch'] > np.radians(20.0):
                    PuppyPose['pitch'] = np.radians(20.0)
            self.PuppyPosePub.publish(Pose(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'],
                                           x_shift=PuppyPose['x_shift'], height=PuppyPose['height'],
                                           roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw']))
            self.get_logger().info(str(PuppyPose))

        if (self.js.get_button_state('L_HAT_UP') or self.js.get_button_state('L_AXIS_UP')
                or self.js.get_button_state('L_HAT_DOWN') or self.js.get_button_state('L_AXIS_DOWN')):
            if kwargs['key'] == 'CIRCLE':
                VelocityX += 0.5
            elif kwargs['key'] == 'SQUARE':
                VelocityX -= 0.5

            if VelocityX > 25.0:
                VelocityX = 25.0
            elif VelocityX < 5.0:
                VelocityX = 5.0

            if PuppyMove['x'] > 0.0:
                PuppyMove['x'] = VelocityX
            elif PuppyMove['x'] < 0.0:
                PuppyMove['x'] = -VelocityX

            self.PuppyVelocityPub.publish(Velocity(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate']))
            self.get_logger().info('HoldFun  %f' % PuppyMove['x'])

    def ReleaseFun(self, *args, **kwargs):
        if kwargs['key'] in PuppyMove.keys():
            PuppyMove[kwargs['key']] = float(args[0])
            self.PuppyVelocityPub.publish(Velocity(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate']))

    def update_buttons(self):
        global enable_control
        with self.js.lock:
            if self.js.js is None or not enable_control:
                return
            # update and read joystick data
            pg.event.pump()
            buttons = [self.js.js.get_button(i) for i in range(13)]
            hat = list(self.js.js.get_hat(0))
            axis = [self.js.js.get_axis(i) for i in range(4)]
            hat.extend(axis)
            # convert analog data to digital
            for i in range(6):
                buttons.extend([1 if hat[i] < -0.5 else 0, 1 if hat[i] > 0.5 else 0])
            # check what has changed in this update
            buttons = np.array(buttons)
            buttons_changed = np.bitwise_xor(self.js.last_buttons, buttons).tolist()
            self.js.last_buttons = buttons  # save buttons data

        for i, button in enumerate(buttons_changed):
            if button:  # button state changed
                if buttons[i]:
                    self.get_logger().debug(BUTTONS[i] + " pressed")
                    self.js.hold_count[i] = 0
                    button_name = BUTTONS[i]
                    if button_name in PRESSED_ACTION_MAP:
                        try:
                            PRESSED_ACTION_MAP[button_name]()
                        except Exception as e:
                            self.get_logger().error(str(e))
                else:
                    self.get_logger().debug(BUTTONS[i] + " released")
                    button_name = BUTTONS[i]
                    if button_name in RELEASED_ACTION_MAP:
                        try:
                            RELEASED_ACTION_MAP[button_name]()
                        except Exception as e:
                            self.get_logger().error(str(e))
            else:
                if buttons[i]:
                    if self.js.hold_count[i] < 3:  # Better distinguish between short press and long press
                        self.js.hold_count[i] += 1
                    else:
                        self.get_logger().debug(BUTTONS[i] + " hold")
                        button_name = BUTTONS[i]
                        if button_name in HOLD_ACTION_MAP:
                            try:
                                HOLD_ACTION_MAP[button_name]()
                            except Exception as e:
                                self.get_logger().error(str(e))


class Joystick:
    def __init__(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
        pg.display.init()

        self.js = None
        self.last_buttons = [0] * len(BUTTONS)
        self.hold_count = [0] * len(BUTTONS)
        self.lock = threading.Lock()
        threading.Thread(target=self.connect, daemon=True).start()

    def get_button_state(self, button):
        return self.last_buttons[BUTTONS.index(button)]

    def connect(self):
        while True:
            if os.path.exists("/dev/input/js0"):
                with self.lock:
                    if self.js is None:
                        pg.joystick.init()
                        try:
                            self.js = pg.joystick.Joystick(0)
                            self.js.init()
                        except Exception as e:
                            print(str(e))
                            self.js = None
            else:
                with self.lock:
                    if self.js is not None:
                        self.js.quit()
                        self.js = None
            time.sleep(0.2)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
