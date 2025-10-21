#!/usr/bin/python3
# coding=utf8
# Date:2021/12/13
# Author:hiwonder
import sys, os
import cv2
import math
import rospy
# import threading
# import numpy as np
from threading import RLock, Timer
from std_msgs.msg import String
from std_srvs.srv import *
# from sensor_msgs.msg import Image
from object_tracking.srv import SetTarget

from puppy_control.msg import Velocity, Pose, Gait
from puppy_control.srv import SetRunActionName



GaitConfigCrawl = {'overlap_time':0.4, 'swing_time':0.3, 'clearance_time':0.20, 'z_clearance':3}
StandLow = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-7, 'x_shift':-0.5, 'stance_x':0, 'stance_y':0}
Stand = {'roll':math.radians(0), 'pitch':math.radians(0), 'yaw':0.000, 'height':-10, 'x_shift':0.4, 'stance_x':1, 'stance_y':0}
PuppyMove = {'x':0, 'y':0, 'yaw_rate':0}


ROS_NODE_NAME = 'performance'


is_self_balancing = False

lock = RLock()
# detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())

# 变量重置(variable reset)
def reset():
    global is_self_balancing
    with lock:
        is_self_balancing = False
        set_self_balancing_srv(is_self_balancing)


# 初始位置(initial position)
def initMove(delay=False):
    # runActionGroup('stand.d6ac')
    go_home_srv()
    with lock:
        pass
    if delay:
        rospy.sleep(2)

# app初始化调用(app initialization calling)
def init():
    print("Init performance")
    initMove(False)
    reset()


def enter_func(msg):
    global lock
    rospy.loginfo("enter performance")
    with lock:
        init()

    return [True, 'enter']

heartbeat_timer = None
def exit_func(msg):
    global lock
    global is_self_balancing

    rospy.loginfo("exit performance")
    with lock:
        is_self_balancing = False

        set_self_balancing_srv(is_self_balancing)
        if heartbeat_timer:heartbeat_timer.cancel()

    return [True, 'exit']


def set_self_balancing(msg):
    global is_self_balancing
    if msg.data:
        is_self_balancing = True
        runActionGroup_srv('stand.d6ac',True)
        # start_running()
    else:
        is_self_balancing = False
        # stop_running()

    set_self_balancing_srv(is_self_balancing)
        

    return [True, 'set_self_balancing']


def set_target(msg):
    global PuppyPose, GaitConfig
    rospy.loginfo("%s", msg)
    with lock:
        if is_self_balancing == False:
            if msg.data == '4_legs_stand':
                # msg.data = 'stand'
                go_home_srv()

            elif msg.data == 'mark_time':#原地踏步(stepping)
                set_mark_time_srv(True)
                rospy.sleep(2)
                go_home_srv()

            elif msg.data == 'crawl': #匍匐前进(crawl forward)
                GaitConfig = GaitConfigCrawl.copy()
                PuppyGaitConfigPub.publish(overlap_time = GaitConfig['overlap_time'], swing_time = GaitConfig['swing_time']
                                , clearance_time = GaitConfig['clearance_time'], z_clearance = GaitConfig['z_clearance'])
                rospy.sleep(0.005)
                
                PuppyPose = StandLow.copy()
                PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                            ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'], run_time = 300)
                rospy.sleep(0.3)

                PuppyMove['x'] = 5
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
                rospy.sleep(3)
                PuppyMove['x'] = 0
                PuppyVelocityPub.publish(x=PuppyMove['x'], y=PuppyMove['y'], yaw_rate=PuppyMove['yaw_rate'])
                
            elif msg.data == 'turn_pitch' or msg.data == 'turn_roll' or msg.data == 'multiaxial': #
                go_home_srv()
                PuppyPose = Stand.copy()

                angle_now = 0
                angle_max = 16
                angle_step = 0.32#0.3
                delay_time = 0.006#0.012
                times = 0
                while True:
                    if msg.data == 'turn_pitch' or msg.data == 'turn_roll':
                        angle_max = 23
                        if times % 3 == 0:
                            angle_now -= math.radians(angle_step)
                            if angle_now <= math.radians(-angle_max):
                                times += 1
                        elif times % 3 == 1:
                            angle_now += math.radians(angle_step)
                            if angle_now >= math.radians(angle_max):
                                times += 1
                        elif times % 3 == 2:
                            angle_now -= math.radians(angle_step)
                            if angle_now <= math.radians(0):
                                times += 1
                                if times == 6:
                                    break
                        if msg.data == 'turn_pitch':PuppyPose['pitch'] = angle_now
                        elif msg.data == 'turn_roll':PuppyPose['roll'] = angle_now
                        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
                        rospy.sleep(delay_time)
                    elif msg.data == 'multiaxial':
                        if times == 12:
                            PuppyPose['roll'] += math.radians(angle_step)
                            if PuppyPose['roll'] >= math.radians(0):
                                break
                        elif times % 6 == 0:
                            PuppyPose['roll'] -= math.radians(angle_step)
                            if PuppyPose['roll'] <= math.radians(-angle_max):
                                times += 1
                        elif times % 6 == 1:
                            PuppyPose['pitch'] -= math.radians(angle_step)
                            if PuppyPose['pitch'] <= math.radians(-angle_max):
                                times += 1
                        elif times % 6 == 2:
                            PuppyPose['roll'] += math.radians(angle_step)
                            if PuppyPose['roll'] >= math.radians(angle_max):
                                times += 1
                        elif times % 6 == 3:
                            PuppyPose['pitch'] += math.radians(angle_step)
                            if PuppyPose['pitch'] >= math.radians(angle_max):
                                times += 1
                        elif times % 6 == 4:
                            PuppyPose['roll'] -= math.radians(angle_step)
                            if PuppyPose['roll'] <= math.radians(-angle_max):
                                times += 1
                        elif times % 6 == 5:
                            PuppyPose['pitch'] -= math.radians(angle_step)
                            if PuppyPose['pitch'] <= math.radians(0):
                                times += 1         
                            

                        PuppyPosePub.publish(stance_x=PuppyPose['stance_x'], stance_y=PuppyPose['stance_y'], x_shift=PuppyPose['x_shift']
                                    ,height=PuppyPose['height'], roll=PuppyPose['roll'], pitch=PuppyPose['pitch'], yaw=PuppyPose['yaw'])
                        rospy.sleep(delay_time)

            else:
                runActionGroup_srv(msg.data + '.d6ac',False)

    return [True, 'set_target']


def heartbeat_srv_cb(msg):
    global heartbeat_timer
    
    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/%s/exit'%ROS_NODE_NAME, Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp

if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)

    PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
    PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
    PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

    enter_srv = rospy.Service('/%s/enter'%ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit'%ROS_NODE_NAME, Trigger, exit_func)
    heartbeat_srv = rospy.Service('/%s/heartbeat'%ROS_NODE_NAME, SetBool, heartbeat_srv_cb)
    go_home_srv = rospy.ServiceProxy('/puppy_control/go_home', Empty)


    rospy.Service('/%s/set_self_balancing'%ROS_NODE_NAME, SetBool, set_self_balancing)
    rospy.Service('/%s/set_target'%ROS_NODE_NAME, SetTarget, set_target)

    rospy.Subscriber('/%s/set_target' %ROS_NODE_NAME, String, set_target)

    runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
    set_self_balancing_srv = rospy.ServiceProxy('/puppy_control/set_self_balancing', SetBool)
    set_mark_time_srv = rospy.ServiceProxy('/puppy_control/set_mark_time', SetBool)

    debug = False
    if debug:
        enter_func(1)
        # start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
