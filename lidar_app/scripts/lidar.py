#!/usr/bin/python3
# coding=utf8
# Date:2021/09/27
# Author:hiwonder

import math
import time
import rospy
from threading import RLock, Timer
import numpy as np
from std_msgs.msg import Float32
import geometry_msgs.msg as geo_msg
import sensor_msgs.msg as sensor_msg
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Empty, Trigger, TriggerRequest, TriggerResponse
from puppy_control.srv import SetInt64, SetInt64Request, SetInt64Response
from puppy_control.srv import SetFloat64List, SetFloat64ListRequest, SetFloat64ListResponse


ROS_NODE_NAME = 'lidar_app'

MAX_SCAN_ANGLE = 360 # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of the laser, excluding the constantly obstructed part, in degrees)





def reset_value():
    node.running_mode = 0
    node.threshold = 0.3
    node.speed = 0.12
    node.scan_angle = math.radians(90)

    try:
        if node.lidar_sub is not None:
            node.lidar_sub.unregister()
    except Exception as e:
        rospy.logerr(str(e))

def enter_func(msg):
    rospy.loginfo("lidar enter")
    reset_value()
    # start_scan()
    rospy.ServiceProxy('/puppy_control/go_home', Empty)()
    node.lidar_sub = rospy.Subscriber('/scan', sensor_msg.LaserScan, node.lidar_callback) 
    return TriggerResponse(success=True)

heartbeat_timer = None
def exit_func(msg):
    global heartbeat_timer
    rospy.loginfo('lidar exit')
    reset_value()
    heartbeat_timer.cancel()
    return TriggerResponse(success=True)

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

class LidarController:
    def __init__(self, name):
        rospy.init_node(name, anonymous=True,log_level=rospy.DEBUG)
        # rospy.on_shutdown(self.cleanup)
        self.name = name
        self.running_mode = 0 # 1：雷达避障模式  2：雷达警卫模式，(1: Radar obstacle avoidance mode 2: Radar guard mode)
        self.threshold = 0.3 # meters  距离阈值(distance threshold)
        self.scan_angle = math.radians(90)  # radians  向前的扫描角度(the forward scanning angle)
        self.speed = 0.12 # 单位米，避障模式的速度(the speed in meters per second for obstacle avoidance mode)
        self.last_act = 0
        self.timestamp = 0
        self.lock = RLock()
        self.lidar_sub = None
        self.velocity_pub = rospy.Publisher('/cmd_vel_nav', geo_msg.Twist, queue_size=1)
        self.velocity_pub.publish(geo_msg.Twist())
        # self.lidar_sub = rospy.Subscriber('/scan', sensor_msg.LaserScan, self.lidar_callback) 

    def lidar_callback(self, lidar_data:sensor_msg.LaserScan):
        ranges = list(lidar_data.ranges)
        ranges = list(9999 if r < 0.05 else r for r in ranges) # 小于5cm当作无限远(treat distances less than 5cm as infinity)
        twist = geo_msg.Twist()

        with self.lock:
            min_index = np.nanargmin(np.array(ranges)) # 找出距离最小值(find out the minimum value of distance)
            dist = ranges[min_index]
            angle = lidar_data.angle_min + lidar_data.angle_increment * min_index # 计算最小值对应的角度(calculate the angle corresponding to the minimum value)
            #雷达朝狗的正前方为0度，然后逆时针旋转角度递增，直到360度(与0度重合)(the radar facing directly towards the dog is at 0 degrees, and then the angle increases in a counterclockwise direction until reaching 360 degrees (coinciding with 0 degrees again))
            angle = angle if angle < math.pi else angle - math.pi*2 
            #将0~360处理成：狗的左边为0~正180度，右边为0~负180度(transforming 0 to 360 degrees: 0 to positive 180 degrees represents the left side of the dog, while 0 to negative 180 degrees represents the right side)
            
            #避障(obstacle avoidance)
            if self.running_mode == 1 and self.timestamp <= time.time():
                if abs(angle) < self.scan_angle/2 and dist < self.threshold:
                    twist.linear.x = self.speed / 6
                    twist.angular.z = self.speed * 3 * -np.sign(angle)
                    self.timestamp = time.time() + 0.8

                else:
                    twist.linear.x = self.speed
                    twist.angular.z = 0
                self.velocity_pub.publish(twist)
 
            # 追踪(tracking)
            elif self.running_mode == 2 and self.timestamp <= time.time():

                if abs(angle) < self.scan_angle/2:
                    if dist < self.threshold and abs(math.degrees(angle)) > 10: # 控制左右(control the left and the right)
                        twist.linear.x = 0.01 # x方向的校正(correction in the x-direction)
                        twist.angular.z = self.speed * 3 * np.sign(angle)
                        self.timestamp = time.time() + 0.4
                    else:
                        if dist < self.threshold and dist > 0.35:
                            twist.linear.x = self.speed
                            twist.angular.z = 0
                            self.timestamp = time.time() + 0.4
                        else:
                            twist.linear.x = 0
                            twist.angular.z = 0
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0
                self.velocity_pub.publish(twist)
            
            # 警卫看守(guard duty)
            elif self.running_mode == 3 and self.timestamp <= time.time():

                if dist < self.threshold and abs(math.degrees(angle)) > 10 :
                    #and abs(angle) < self.scan_angle/2: # 控制左右(control the left and the right)
                    twist.linear.x = 0.01 # x方向的校正(correction in the x-direction)
                    twist.angular.z = self.speed * 3 * np.sign(angle)
                    self.timestamp = time.time() + 0.4
                else:
                    twist.linear.x = 0
                    twist.angular.z = 0
                self.velocity_pub.publish(twist)


# 设置运行模式(set the running mode)
def set_running_srv_callback(req: SetInt64Request):
    rsp =SetInt64Response(success=True)
    new_running_mode = req.data
    rospy.loginfo("set_running " + str(new_running_mode))
    if not 0 <= new_running_mode <= 3:
        rsp.success = False
        rsp.message = "Invalid running mode {}".format(new_running_mode)
    else:
        # with self.lock:
        node.running_mode = new_running_mode
        # if node.running_mode == 0:
        #     node.jethexa.traveling(gait=-2)
    node.velocity_pub.publish(geo_msg.Twist())
    return rsp

# 设置运行参数(set the running parameter)
def set_parameters_srv_callback(req: SetFloat64ListRequest):
    rsp = SetFloat64ListResponse(success=True)
    new_parameters = req.data
    new_threshold, new_scan_angle, new_speed = new_parameters
    rospy.loginfo("n_t:{:2f}, n_a:{:2f}, n_s:{:2f}".format(new_threshold, new_scan_angle, new_speed))
    if not 0.3 <= new_threshold <= 1.5:
        rsp.success = False
        rsp.message = "New threshold ({:.2f}) is out of range (0.3 ~ 1.5)".format(new_threshold)
        return rsp
    """
    if not 0 <= new_scan_angle <= 90:
        rsp.success = False
        rsp.message = "New scan angle ({:.2f}) is out of range (0 ~ 90)"
        return rsp
    """
    if not new_speed > 0:
        rsp.success = False
        rsp.message = "Invalid speed"
        return rsp

    # with self.lock:
    node.threshold = new_threshold
    #self.scan_angle = math.radians(new_scan_angle)
    node.speed = new_speed * 0.002 
    
    return rsp
    
if __name__ == "__main__":
    node = LidarController(ROS_NODE_NAME)

    enter_srv = rospy.Service('/%s/enter'%ROS_NODE_NAME, Trigger, enter_func)
    exit_srv = rospy.Service('/%s/exit'%ROS_NODE_NAME, Trigger, exit_func)
    heartbeat_srv = rospy.Service('/%s/heartbeat'%ROS_NODE_NAME, SetBool, heartbeat_srv_cb)

    set_running_srv = rospy.Service("/%s/set_running"%ROS_NODE_NAME, SetInt64, set_running_srv_callback)
    set_parameters_srv = rospy.Service("/%s/set_parameters"%ROS_NODE_NAME, SetFloat64List, set_parameters_srv_callback)

    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

