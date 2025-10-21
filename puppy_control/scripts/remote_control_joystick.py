#!/usr/bin/env python3
# coding=utf8
import os
import sys
import rospy
import threading
import math
import numpy as np
import pygame as pg
from functools import partial

from ros_robot_controller.msg import BuzzerState
from geometry_msgs.msg import Polygon
from puppy_control.msg import Velocity, Pose
from std_msgs.msg import Float32
from std_srvs.srv import Empty, SetBool


# PuppyPose

PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}
Stand = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}

ROS_NODE_NAME = 'remote_control_joystick'


enable_control = True

PRESSED_ACTION_MAP = {}
HOLD_ACTION_MAP = {}
RELEASED_ACTION_MAP = {}
BUTTONS = ("CROSS", "CIRCLE", "None_1", "SQUARE",
            "TRIANGLE", "None_2", "L1", "R1",
            "L2", "R2", "SELECT", "START", "MODE",
            "L_HAT_LEFT", "L_HAT_RIGHT", "L_HAT_DOWN", "L_HAT_UP",
            "L_AXIS_LEFT", "L_AXIS_RIGHT", "L_AXIS_UP", "L_AXIS_DOWN",
            "R_AXIS_LEFT", "R_AXIS_RIGHT", "R_AXIS_UP", "R_AXIS_DOWN")
#BUTTONS = ("TRIANGLE", "CIRCLE", "CROSS",  "SQUARE","L1", "R1", "L2", "R2", 
#            "SELECT", "START", "L_AXIS_BUTTON", "R_AXIS_BUTTON", "MODE",
#           "L_HAT_LEFT", "L_HAT_RIGHT", "L_HAT_DOWN", "L_HAT_UP",
#           "L_AXIS_LEFT", "L_AXIS_RIGHT", "L_AXIS_UP", "L_AXIS_DOWN",
#           "R_AXIS_LEFT", "R_AXIS_RIGHT", "R_AXIS_UP", "R_AXIS_DOWN")




LegsCoord = None

VelocityX = 15



def go_home():
    global PuppyPose, PuppyMove
    print('go_home')
    # PuppyPose = PP.Stand.copy()
    PuppyPose = Stand.copy()
    PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}
    rospy.ServiceProxy('/puppy_control/go_home', Empty)()
    msg = BuzzerState()
    msg.freq = 1900
    msg.on_time = 0.1
    msg.off_time = 0.9
    msg.repeat = 1
    buzzer_pub.publish(msg)
    rospy.loginfo(PuppyPose)


def LegsCoordFun(msg):
    global LegsCoord
    LegsCoord = msg


def PressedFun(*args,**kwargs):
    global LegsCoord
    global VelocityX
    # print('PressedFun',args,kwargs) 

    if (js.get_button_state('L_HAT_UP') or js.get_button_state('L_AXIS_UP') 
        or js.get_button_state('L_HAT_DOWN') or js.get_button_state('L_AXIS_DOWN')
        ):
        if kwargs['key'] == 'CIRCLE':
            VelocityX += 1
        elif kwargs['key'] == 'SQUARE':
            VelocityX -= 1

        if VelocityX > 25:VelocityX = 25
        elif VelocityX < 5:VelocityX = 5

        if PuppyMove['x'] > 0:
            PuppyMove['x'] = VelocityX
        elif PuppyMove['x'] < 0:
            PuppyMove['x'] = -VelocityX
        PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
        rospy.loginfo(PuppyMove['x'])

    if kwargs['key'] in PuppyMove.keys():
        # print(kwargs['key'],args[0])

        if kwargs['key'] == 'x':
            if args[0] > 0:
                PuppyMove['x'] = VelocityX
            else:
                PuppyMove['x'] = -VelocityX
        else:
            PuppyMove[kwargs['key']] = args[0]
        PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
        rospy.loginfo(PuppyMove['x'])
    elif kwargs['key'] in PuppyPose.keys(): 
        if kwargs['key'] == 'height':
            PuppyPose['height'] += args[0]
            if PuppyPose['height'] < -16:PuppyPose['height'] = -16
            if PuppyPose['height'] > -5:PuppyPose['height'] = -5
        if kwargs['key'] == 'x_shift':
            PuppyPose['x_shift'] += args[0]
            if PuppyPose['x_shift'] < -6:PuppyPose['x_shift'] = -6
            if PuppyPose['x_shift'] > 6:PuppyPose['x_shift'] = 6
        if kwargs['key'] == 'pitch':
            PuppyPose['pitch'] += args[0]
            if PuppyPose['pitch'] < -np.radians(20):PuppyPose['pitch'] = -np.radians(20)
            if PuppyPose['pitch'] > np.radians(20):PuppyPose['pitch'] = np.radians(20)
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
        rospy.loginfo(PuppyPose)
    elif kwargs['key'] == 'legs_coord':
        print(LegsCoord)


def HoldFun(*args,**kwargs):
    global VelocityX
    # rospy.logdebug('HoldFun',args,kwargs)
    if kwargs['key'] in PuppyPose.keys(): 
        if kwargs['key'] == 'height':
            PuppyPose['height'] += args[0]
            if PuppyPose['height'] < -16:PuppyPose['height'] = -16
            if PuppyPose['height'] > -5:PuppyPose['height'] = -5
        if kwargs['key'] == 'x_shift':
            PuppyPose['x_shift'] += args[0]
            if PuppyPose['x_shift'] < -6:PuppyPose['x_shift'] = -6
            if PuppyPose['x_shift'] > 6:PuppyPose['x_shift'] = 6
        if kwargs['key'] == 'pitch':
            PuppyPose['pitch'] += args[0]
            if PuppyPose['pitch'] < -np.radians(20):PuppyPose['pitch'] = -np.radians(20)
            if PuppyPose['pitch'] > np.radians(20):PuppyPose['pitch'] = np.radians(20)
        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
        rospy.loginfo(PuppyPose)

    if (js.get_button_state('L_HAT_UP') or js.get_button_state('L_AXIS_UP') 
        or js.get_button_state('L_HAT_DOWN') or js.get_button_state('L_AXIS_DOWN')
        ):
        if kwargs['key'] == 'CIRCLE':
            VelocityX += 0.5
        elif kwargs['key'] == 'SQUARE':
            VelocityX -= 0.5

        if VelocityX > 25:VelocityX = 25
        elif VelocityX < 5:VelocityX = 5

        if PuppyMove['x'] > 0:
            PuppyMove['x'] = VelocityX
        elif PuppyMove['x'] < 0:
            PuppyMove['x'] = -VelocityX

        PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
        rospy.loginfo('HoldFun  %f'% PuppyMove['x'])

def ReleaseFun(*args,**kwargs):
    if kwargs['key'] in PuppyMove.keys():
        PuppyMove[kwargs['key']] = args[0]
        PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])

PRESSED_ACTION_MAP = {
    "START": go_home,
    "SELECT": partial(PressedFun, 0, key = 'legs_coord'),

    "L_HAT_UP": partial(PressedFun, 1, key = 'x'),
    "L_AXIS_UP": partial(PressedFun, 1, key = 'x'),
    "L_HAT_DOWN": partial(PressedFun, -1, key = 'x'),
    "L_AXIS_DOWN": partial(PressedFun, -1, key = 'x'),
    "L_HAT_LEFT": partial(PressedFun, np.radians(20), key = 'yaw_rate'),
    "L_AXIS_LEFT": partial(PressedFun, np.radians(20), key = 'yaw_rate'),
    "L_HAT_RIGHT": partial(PressedFun, np.radians(-20), key = 'yaw_rate'),
    "L_AXIS_RIGHT": partial(PressedFun, np.radians(-20), key = 'yaw_rate'),

    "TRIANGLE": partial(PressedFun, -0.15, key = 'height'),
    "CROSS": partial(PressedFun, 0.15, key = 'height'),
    "SQUARE": partial(PressedFun, 0.15, key = 'SQUARE'),
    "CIRCLE": partial(PressedFun, 0.15, key = 'CIRCLE'),

    "L1": partial(PressedFun, 0.015, key = 'pitch'),
    "L2": partial(PressedFun, -0.015, key = 'pitch'),
    "R1": partial(PressedFun, -0.15, key = 'x_shift'),
    "R2": partial(PressedFun, 0.15, key = 'x_shift'),
}

HOLD_ACTION_MAP = {
    "TRIANGLE": partial(HoldFun, -0.15, key = 'height'),
    "CROSS": partial(HoldFun, 0.15, key = 'height'),
    "SQUARE": partial(HoldFun, 0.15, key = 'SQUARE'),
    "CIRCLE": partial(HoldFun, 0.15, key = 'CIRCLE'),

    "L1": partial(HoldFun, 0.015, key = 'pitch'),
    "L2": partial(HoldFun, -0.015, key = 'pitch'),
    "R1": partial(HoldFun, -0.15, key = 'x_shift'),
    "R2": partial(HoldFun, 0.15, key = 'x_shift'),
}
RELEASED_ACTION_MAP = {
    "L_HAT_UP": partial(ReleaseFun, 0, key = 'x'),
    "L_AXIS_UP": partial(ReleaseFun, 0, key = 'x'),
    "L_HAT_DOWN": partial(ReleaseFun, 0, key = 'x'),
    "L_AXIS_DOWN": partial(ReleaseFun, 0, key = 'x'),
    "L_HAT_LEFT": partial(ReleaseFun, np.radians(0), key = 'yaw_rate'),
    "L_AXIS_LEFT": partial(ReleaseFun, np.radians(0), key = 'yaw_rate'),
    "L_HAT_RIGHT": partial(ReleaseFun, np.radians(0), key = 'yaw_rate'),
    "L_AXIS_RIGHT": partial(ReleaseFun, np.radians(0), key = 'yaw_rate'),
}


class Joystick:
    def __init__(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
        pg.display.init()

        self.js = None
        self.last_buttons = [0] * len(BUTTONS)
        self.hold_count = [0] * len(BUTTONS)
        self.lock = threading.Lock()
        self.speed_x = 0
        self.speed_y = 0
        threading.Thread(target=self.connect, daemon=True).start()

    def get_button_state(self,button):
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
                            rospy.logerr(e)
                            self.js = None
            else:
                with self.lock:
                    if self.js is not None:
                        self.js.quit()
                        self.js = None
            rospy.sleep(0.2)

    def update_buttons(self):
        global enable_control
        with self.lock:
            if self.js is None or not enable_control:
                return
            # update and read joystick data
            pg.event.pump()
            buttons = [self.js.get_button(i) for i in range(13)]
            # print(buttons)
            hat = list(self.js.get_hat(0))
            axis = [self.js.get_axis(i) for i in range(4)]
            hat.extend(axis)
            # convert analog data to digital
            for i in range(6):
                buttons.extend([1 if hat[i] < -0.5 else 0, 1 if hat[i] > 0.5 else 0])
            # check what has changed in this update
            buttons = np.array(buttons)
            buttons_changed = np.bitwise_xor(self.last_buttons, buttons).tolist()
            self.last_buttons = buttons  # save buttons data

        # if chassis:
        #     axis[0] = hiwonder.misc.val_map(axis[0], -1, 1, -200, 200)
        #     axis[1] = hiwonder.misc.val_map(axis[1], -1, 1, 200, -200)
        #     axis[2] = hiwonder.misc.val_map(axis[2], -1, 1, 90, -90)
        #     v, d = chassis.translation(axis[0], axis[1], fake=True)
        #     chassis.set_velocity(v, d, axis[2])

        for i, button in enumerate(buttons_changed):
            if button:  # button state changed
                if buttons[i]:
                    rospy.logdebug(BUTTONS[i] + " pressed")
                    self.hold_count[i] = 0
                    button_name = BUTTONS[i]
                    if button_name in PRESSED_ACTION_MAP:
                        try:
                            PRESSED_ACTION_MAP[button_name]()
                        except Exception as e:
                                rospy.logerr(e)
                else:
                    rospy.logdebug(BUTTONS[i] + " released")
                    button_name = BUTTONS[i]
                    if button_name in RELEASED_ACTION_MAP:
                        try:
                            RELEASED_ACTION_MAP[button_name]()
                        except Exception as e:
                                rospy.logerr(e)
            else:
                if buttons[i]:
                    if self.hold_count[i] < 3:  # Better distinguish between short press and long press
                        self.hold_count[i] += 1
                    else:
                        rospy.logdebug(BUTTONS[i] + " hold")
                        button_name = BUTTONS[i]
                        if button_name in HOLD_ACTION_MAP:
                            try:
                                HOLD_ACTION_MAP[button_name]()
                            except Exception as e:
                                rospy.logerr(e)




if __name__ == '__main__':

   #rospy.sleep(4)#等待参数加载完毕(wait for the parameters to load completely)
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.INFO)
    #PP = rospy.get_param('/puppy_control/PuppyPose')
    PuppyPose = Stand.copy()

    rospy.Subscriber('/puppy_control/legs_coord', Polygon, lambda msg:LegsCoordFun(msg))
    buzzer_pub = rospy.Publisher('/ros_robot_controller/set_buzzer', BuzzerState, queue_size=1)

    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity/autogait', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

    js = Joystick()

    while True:
        try:
            js.update_buttons()
            rospy.sleep(0.05)
            if rospy.is_shutdown():
                sys.exit(0)
        except KeyboardInterrupt:
            sys.exit(0)
