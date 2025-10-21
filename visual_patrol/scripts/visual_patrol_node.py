#!/usr/bin/python3
# coding=utf8
# Date:2021/12/02
# Author:hiwonder
import sys
import cv2
import math
import rospy
import threading
import numpy as np
from threading import RLock, Timer
from std_srvs.srv import *
from sensor_msgs.msg import Image
from object_tracking.srv import *
from ros_robot_controller.msg import RGBState,RGBsState
from puppy_control.msg import Velocity, Pose, Gait
from visual_patrol.srv import SetPoint, SetPointRequest, SetPointResponse
from visual_patrol.srv import SetFloat64, SetFloat64Request, SetFloat64Response


from common import Misc
#from common.color_picker import ColorPicker
from visual_patrol_app.common import ColorPicker

class LineFollower:
    def __init__(self, color, node):
        self.node = node
        self.target_lab, self.target_rgb = color
        #self.rois = ((240, 280,  0, 640, 0.1), (320, 360,  0, 640, 0.2), (400, 440,  0, 640, 0.7))
        self.rois = ((120, 140,  0, 320, 0.1), (160, 180,  0, 320, 0.2), (190, 210,  0, 320, 0.7))
        
        roi_h1 = self.rois[0][0]
        roi_h2 = self.rois[1][0] - self.rois[0][0]
        roi_h3 = self.rois[2][0] - self.rois[1][0]
        self.roi_h_list = [roi_h1, roi_h2, roi_h3]
              
        

    @staticmethod
    def get_area_max_contour(contours, threshold=100):
        '''
        获取最大面积对应的轮廓(get the contour of the largest area)
        :param contours:
        :param threshold:
        :return:
        '''
        contour_area = zip(contours, tuple(map(lambda c: math.fabs(cv2.contourArea(c)), contours)))
        contour_area = tuple(filter(lambda c_a: c_a[1] > threshold, contour_area))        
        if len(contour_area) > 0:
            max_c_a = max(contour_area, key=lambda c_a: c_a[1])
            return max_c_a
        return None

    def __call__(self, image, result_image, threshold):
        centroid_sum = 0
        weight_sum = 0
        line_center_x = -1
        line_center_y = -1
        size = (320, 240)
        h, w = image.shape[:2]
        image_copy = image.copy()
        frame_resize = cv2.resize(image_copy, size, interpolation=cv2.INTER_NEAREST)
        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color      
        
        n = 0
        
        for roi in self.rois:  
            roi_h = self.roi_h_list[n]
            n += 1       
            if n <= 2:
                 continue                                 
            blob = frame_resize[roi[0]:roi[1], roi[2]:roi[3]]  # 截取roi(intercept roi)
            img_lab = cv2.cvtColor(blob, cv2.COLOR_RGB2LAB)  # rgb转lab(convert rgb into lab)
            img_blur = cv2.GaussianBlur(img_lab, (3, 3), 3)  # 高斯模糊去噪(perform Gaussian filtering to reduce noise)
            mask = cv2.inRange(img_blur, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(image binarization)
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(corrode)
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
            # cv2.imshow('section:{}:{}'.format(roi[0], roi[1]), cv2.cvtColor(dilated, cv2.COLOR_GRAY2BGR))
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找轮廓(find the contour)
            max_contour_area = self.get_area_max_contour(contours, 50)  # 获取最大面积对应轮廓(get the contour corresponding to the largest contour)
            if max_contour_area is not None:
                rect = cv2.minAreaRect(max_contour_area[0])  # 最小外接矩形(minimum circumscribed rectangle)
                box = np.intp(cv2.boxPoints(rect))  # 四个角(four corners)
                for j in range(4):
                    box[j, 1] = box[j, 1] + roi[0]      
                    box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, h)) 
                for j in range(4):  
                    box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, w))                                                        
                cv2.drawContours(result_image, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形(draw the rectangle composed of four points)

                # 获取矩形对角点(acquire the diagonal points of the rectangle)
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                # 线的中心点(center point of the line)                
                line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2

                cv2.circle(result_image, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)   # 画出中心点(draw the center point)        
                centroid_sum += line_center_x * roi[-1]
                weight_sum += roi[-1]                 
            if weight_sum != 0:
                center_pos = centroid_sum / weight_sum  # 按比重计算中心点(calculate the center point according to the ratio)               
            else:
                center_pos = -1
        return result_image, center_pos


class LineFollowingNode:
    def __init__(self, name):
        rospy.init_node(name)              
        self.lock = threading.RLock()
        ROS_NODE_NAME = name
        self.threshold = 0.1
        self.stop_threshold = 0.4
        self.img_centerx = 320
        self.org_image_sub_ed = False
        self.heartbeat_timer = None
        self.__isRunning = False
        
        self.PuppyMove = {'x': 0, 'y': 0, 'yaw_rate': 0}
        self.LookDown_20deg = {'roll':math.radians(0), 'pitch':math.radians(-20), 'yaw':0.000, 'height':-9, 'x_shift':-0.1, 'stance_x':0, 'stance_y':0}
        self.GaitConfigFast = {'overlap_time':0.1, 'swing_time':0.15, 'clearance_time':0.0, 'z_clearance':5}  
        self.PuppyPose = self.LookDown_20deg.copy()
        self.GaitConfig = self.GaitConfigFast.copy()
        
        self.image_pub = rospy.Publisher('/%s/image_result'%ROS_NODE_NAME, Image, queue_size=1)  # register result image publisher
        self.rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)
        
        self.PuppyGaitConfigPub = rospy.Publisher('/puppy_control/gait', Gait, queue_size=1)
        self.PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
        self.PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)

        self.enter_srv = rospy.Service('/%s/enter'%ROS_NODE_NAME, Trigger, self.enter_func)
        self.exit_srv = rospy.Service('/%s/exit'%ROS_NODE_NAME, Trigger, self.exit_func)
        self.running_srv = rospy.Service('/%s/set_running'%ROS_NODE_NAME, SetBool, self.set_running)
        self.set_target_srv = rospy.Service('/%s/set_target_color'%ROS_NODE_NAME, SetPoint, self.set_target)# 设置颜色(set the color)
        self.get_target_color_srv = rospy.Service('/%s/get_target_color'%ROS_NODE_NAME, Trigger, self.get_target_color_srv_callback)   # 获取颜色(get the color)
        self.heartbeat_srv = rospy.Service('/%s/heartbeat'%ROS_NODE_NAME, SetBool, self.heartbeat_srv_cb)
        self.set_threshold_srv = rospy.Service('/%s/set_threshold'%ROS_NODE_NAME, SetFloat64, self.set_threshold_srv_callback)  # 设置阈值(set the threshold)
                
        
    def reset(self,):
        with self.lock:
            self.turn_off_rgb()
            self.line_centerx = -1
            self.follower = None
            self.stop = False
            self.is_running = False
            self.threshold = 0.1
            self.empty = 0
            self.color_picker = None
            #self.draw_color = range_rgb["black"]
            self.PuppyMove['x'] = 0
            self.PuppyMove['yaw_rate'] = math.radians(0)
            self.PuppyVelocityPub.publish(x=self.PuppyMove['x'], y=self.PuppyMove['y'], yaw_rate=self.PuppyMove['yaw_rate'])
         
    def init_move(self, delay = True):
        self.PuppyMove['x'] = 0
        self.PuppyMove['yaw_rate'] = math.radians(0)
        self.PuppyVelocityPub.publish(x=self.PuppyMove['x'], y=self.PuppyMove['y'], yaw_rate=self.PuppyMove['yaw_rate'])

        rospy.ServiceProxy('/puppy_control/go_home', Empty)()
        self.PuppyPosePub.publish(stance_x=self.PuppyPose['stance_x'], stance_y=self.PuppyPose['stance_y'], x_shift=self.PuppyPose['x_shift']
        ,height=self.PuppyPose['height'], roll=self.PuppyPose['roll'], pitch=self.PuppyPose['pitch'], yaw=self.PuppyPose['yaw'], run_time = 500)


        self.PuppyGaitConfigPub.publish(overlap_time = self.GaitConfig['overlap_time'], swing_time = self.GaitConfig['swing_time']
                , clearance_time = self.GaitConfig['clearance_time'], z_clearance = self.GaitConfig['z_clearance'])
                
        if delay:
            rospy.sleep(0.5)
            
    def turn_off_rgb(self,):
        led1 = RGBState()
        led1.id = 1
        led1.r = 0
        led1.g = 0
        led1.b = 0
    
        led2 = RGBState()
        led2.id = 2
        led2.r = 0
        led2.g = 0
        led2.b = 0
        msg = RGBsState()
        msg.data = [led1,led2]
        self.rgb_pub.publish(msg)
        rospy.sleep(0.005)
        
    def turn_on_rgb(self, r, g, b):
    
        led1 = RGBState()
        led1.id = 1
        led1.r = r
        led1.g = g
        led1.b = b
    
        led2 = RGBState()
        led2.id = 2
        led2.r = r
        led2.g = g
        led2.b = b
        msg = RGBsState()
        msg.data = [led1,led2]
        self.rgb_pub.publish(msg)
        rospy.sleep(0.005)

        
    # app初始化调用(app initialization calling)
    def init(self,):
        rospy.loginfo("visual patrol Init")
        #self.init_move(True)
        self.reset()
        
                
    def image_callback(self, ros_image):  
        #rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the customized image information to image)
        
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3),dtype=np.uint8, buffer=ros_image.data)
        #cv2_img = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        
        result_image = np.copy(rgb_image)
        with self.lock:
            # 颜色拾取器和识别巡线互斥, 如果拾取器存在就开始拾取(color picker and line recognition are exclusive. If there is color picker, start picking)
            if self.color_picker is not None:  # 拾取器存在(color picker exists)
                try:                
                    target_color, result_image = self.color_picker(rgb_image, result_image)
                    if target_color is not None:
                        self.color_picker = None  
                                             
                        self.follower = LineFollower(target_color, self) 
                        print("target_color:",*self.follower.target_rgb)
                        self.turn_on_rgb(*self.follower.target_rgb)                        
                except Exception as e:
                    rospy.logerr(str(e))
            else: 
                if self.follower is not None:
                    try:
                        result_image, center_pos = self.follower(rgb_image, result_image, self.threshold)                    
                        if center_pos != -1 and self.__isRunning:
                            if abs(center_pos - self.img_centerx) <= 50:
                                self.PuppyMove['x'] = 10
                                self.PuppyMove['yaw_rate'] = math.radians(0)
                            elif center_pos - self.img_centerx > 50:
                                self.PuppyMove['x'] = 8
                                self.PuppyMove['yaw_rate'] = math.radians(-15)
                            elif center_pos - self.img_centerx < -50:
                                self.PuppyMove['x'] = 8
                                self.PuppyMove['yaw_rate'] = math.radians(15)                           
                            self.PuppyVelocityPub.publish(x=self.PuppyMove['x'], y=self.PuppyMove['y'], yaw_rate=self.PuppyMove['yaw_rate'])
                        else:
                            self.PuppyMove['x'] = 0
                            self.PuppyMove['yaw_rate'] = math.radians(0)
                            self.PuppyVelocityPub.publish(x=self.PuppyMove['x'], y=self.PuppyMove['y'], yaw_rate=self.PuppyMove['yaw_rate'])
                        rospy.sleep(0.02)
                    except Exception as e:
                        rospy.logerr(str(e))
                else:
                    rospy.sleep(0.02)
                    
        ros_image.data = result_image.tobytes()
        self.image_pub.publish(ros_image)
    
    
    def enter_func(self, msg):
        rospy.loginfo("enter visual patrol")  
        
        if not self.org_image_sub_ed:
            self.org_image_sub_ed = True  
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback) 
        self.init()
        self.stop = False
        
        self.init_move(True)
        
                    
                
        return [True, 'enter']
    
    
    
    def exit_func(self, msg):
        rospy.loginfo("exit visual patrol")
        self.__isRunning = False
        rospy.ServiceProxy('/puppy_control/go_home', Empty)()
            
        self.reset()
        try:
            if self.org_image_sub_ed:
                self.org_image_sub_ed = False
                if self.heartbeat_timer:self.heartbeat_timer.cancel()
                self.image_sub.unregister()
        except:
            pass
        
        return [True, 'exit']
    
    
    def start_running():
        rospy.loginfo("start running")
        with self.lock:
            self.__isRunning = True
            self.empty = 0
    
    
    def stop_running(self,):
        rospy.loginfo("stop running")
        with self.lock:
            self.__isRunning = False
            #reset()      
    
    def set_running(self, msg):
        rospy.loginfo("set_running")
        if msg.data:
            self.__isRunning = True
            self.empty = 0
            #self.start_running()
        else:
            self.__isRunning = False
            self.PuppyMove['x'] = 0
            self.PuppyMove['yaw_rate'] = math.radians(0)
            self.PuppyVelocityPub.publish(x=self.PuppyMove['x'], y=self.PuppyMove['y'], yaw_rate=self.PuppyMove['yaw_rate'])
            #self.reset()
            #self.stop_running()
        
        return [True, 'set_running'] 
        
    def set_target(self, req: SetPointRequest):
        #rospy.loginfo("%s", req.data)
        with self.lock:
            x, y = req.data.x, req.data.y
            self.follower = None
            if x == -1 and y == -1:
                self.color_picker = None
            else:
                rospy.loginfo(req.data)
                self.color_picker = ColorPicker(req.data, 20)
                #self.turn_on_rgb(*self.color_picker .target_rgb)
                #a , b = self.color_picker
                rospy.sleep(0.012)

        success=True
        message = "Target set successfully"
            
        return SetPointResponse(success,message) 
        
    def get_target_color_srv_callback(self, _):          
        rospy.loginfo("get_target_color") 
        rsp = TriggerResponse(success=False, message="")
        with self.lock:
            if self.follower is not None:
                rsp.success = True
                rgb = self.follower.target_rgb
                rsp.message = "{},{},{}".format(int(rgb[0]), int(rgb[1]), int(rgb[2]))
        return rsp
    
    def set_threshold_srv_callback(self, req: SetFloat64Request):
        rospy.loginfo("set threshold")
        with self.lock:
            self.threshold = req.data
        return SetFloat64Response(success=True)
    
    def heartbeat_srv_cb(self, msg):
        if isinstance(self.heartbeat_timer, Timer):
            self.heartbeat_timer.cancel()
        if msg.data:
            self.heartbeat_timer = Timer(5, rospy.ServiceProxy('/visual_patrol/exit', Trigger))
            self.heartbeat_timer.start()
        rsp = SetBoolResponse()
        rsp.success = msg.data
    
        return rsp
        


if __name__ == "__main__":
    LineFollowingNode('visual_patrol')
    #LineFollowingNode('line_following')  
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))



