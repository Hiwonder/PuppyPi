# encoding:utf-8
# @data:2024/09/02
import sys
import cv2
import math
import rclpy
import yaml
import array
import queue
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Header
from rclpy.callback_groups import ReentrantCallbackGroup
from interfaces.srv import GetRange, ChangeRange, StashRange, GetAllColorName
import numpy as np
from cv_bridge import CvBridge
from threading import RLock, Timer
import yaml
from interfaces.srv import *
bridge = CvBridge()

lock = RLock()
__isRunning = False
image_sub = None
image_pub = None
kernel_erode = 3
kernel_dilate = 3
config_file_path = '/home/ubuntu/ros2_ws/src/driver/lab_config/config/lab_config.yaml'
heartbeat_timer = None
sub_ed = False

def cv2_image2ros(image):
    image_temp = Image()
    header = Header(stamp=rclpy.time.Time())
    header.frame_id = 'map'
    image_temp.height = image.shape[:2][0]
    image_temp.width = image.shape[:2][1]
    image_temp.encoding = 'rgb8'
    image_temp.data = image.tobytes()
    image_temp.header = header
    image_temp.step = image_temp.width * 3
    return image_temp
    
class LabConfigManager(Node):
    def __init__(self):
        super().__init__('lab_config_manager',allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.__color_range_g = {}
        self.current_range = {'min': [0, 0, 0], 'max': [100, 100, 100]}
        self.color_ranges = self.get_parameters_by_prefix('color_range_list').items()  
        for key, value in self.color_ranges:
            type_ = key[-3:]
            color = key[:-4]
            values = value.value        
            if color not in self.__color_range_g:
                self.__color_range_g[color] = {}                 
            if type_ not in self.__color_range_g[color]:
                self.__color_range_g[color][type_] = []     
            if isinstance(values, int):
                self.__color_range_g[color][type_].append(values)
            else:
                 self.__color_range_g[color][type_].extend(values) 
            
        self.get_logger().info(str(self.__color_range_g))
        config_file_path = self.get_parameter('config_file_path').value
        self.get_logger().info(config_file_path)
       
        kernel_erode = self.get_parameter('kernel_erode').value
        kernel_dilate = self.get_parameter('kernel_dilate').value     
        if 'red' in self.__color_range_g:
            self.current_range = self.__color_range_g['red']
        timer_cb_group = ReentrantCallbackGroup()
        #,callback_group=timer_cb_group
        self.image_pub = self.create_publisher(Image, '/lab_config_manager/image_result', 10)
        self.enter_srv = self.create_service(Trigger, '/lab_config_manager/enter', self.enter_func)
        self.exit_srv = self.create_service(Trigger, '/lab_config_manager/exit', self.exit_func,callback_group=timer_cb_group)
        self.lab_adjust_srv = self.create_service(Trigger, '/lab_config_manager/lab_adjust', self.lab_adjust,callback_group=timer_cb_group)
        self.running_srv = self.create_service(SetBool, '/lab_config_manager/set_running', self.set_running,callback_group=timer_cb_group)
        self.save_to_disk_srv = self.create_service(Trigger, 'lab_config_manager/save_to_disk', self.save_to_disk_srv_cb,callback_group=timer_cb_group)
        self.get_color_range_srv = self.create_service(GetRange, 'lab_config_manager/get_range', self.get_range_srv_cb,callback_group=timer_cb_group)
        self.change_range_srv = self.create_service(ChangeRange, 'lab_config_manager/change_range', self.change_range_srv_cb,callback_group=timer_cb_group)
        self.stash_range_srv = self.create_service(StashRange, 'lab_config_manager/stash_range', self.stash_range_srv_cb,callback_group=timer_cb_group)
        self.get_all_color_name_srv = self.create_service(GetAllColorName, '/lab_config_manager/get_all_color_name', self.get_all_color_name_srv_cb,callback_group=timer_cb_group)
        #self.heartbeat_srv = self.create_service(SetBool, 'lab_config_manager/heartbeat', self.heartbeat_srv_cb)

    def image_callback(self, ros_image):
        global lock, __isRunning
        '''
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)
        '''
        rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='rgb8')
        
        
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        
        #with lock:            
        image =  rgb_image        
        image_resize = cv2.resize(image, (400, 300), interpolation=cv2.INTER_NEAREST)
        frame_result = cv2.cvtColor(image_resize, cv2.COLOR_RGB2LAB)
        range_ = self.current_range
        mask = cv2.inRange(frame_result, tuple(range_['min']), tuple(range_['max']))
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_erode, kernel_erode)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_dilate, kernel_dilate)))
        gray_image = cv2.cvtColor(dilated, cv2.COLOR_GRAY2RGB)
        rgb_image = np.vstack((gray_image, image_resize))           
        ros_img = self.bridge.cv2_to_imgmsg(rgb_image, encoding='bgr8')            
        self.image_pub.publish(ros_img)
    def enter_func(self, request, response):
        global lock, image_sub, sub_ed
        with lock:
            if not sub_ed:
                sub_ed = True
                image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        response.success = True
        response.message = 'enter'       
        return response

    def lab_adjust(self, request, response):
        response.success = True
        response.message = 'lab_adjust'
        return response

    def exit_func(self, request, response):
        global lock, image_sub, __isRunning, sub_ed
        with lock:
            __isRunning = False
            if sub_ed:
                sub_ed = False
                image_sub.destroy()
        response.success = True
        response.message = 'exit'
        return response

    def start_running(self):
        global lock, __isRunning
        with lock:
            __isRunning = True
        self.get_logger().info("open success")

    def stop_running(self):
        global lock, __isRunning
        with lock:
            __isRunning = False

    def set_running(self, request, response):
        if request.data:
            self.start_running()
        else:
            self.stop_running()
        response.success = True
        response.message = 'set_running'
        return response

    def save_to_disk_srv_cb(self, request, response):
        global lock
        with lock:
            cf = {"color_range_list": self.__color_range_g.copy()}
            self.get_logger().info(str(cf))
        s = yaml.dump(cf, default_flow_style=False)
        with open(config_file_path, 'w') as f:
            f.write(s)          
        response.success = True
        return response

    def get_range_srv_cb(self, request, response):
        global lock
        ranges = [i[:-4] for i,j in self.get_parameters_by_prefix('color_range_list').items()  ]
        
        ranges = set(ranges) or self.__color_range_g
        
        with lock:
            ranges = self.__color_range_g
        if request.color_name in ranges:
            response.success = True
            response.min = ranges[request.color_name]['min']
            response.max = ranges[request.color_name]['max']
        else:
            response.success = False
        return response

    def change_range_srv_cb(self, request, response):
        with lock:
            self.current_range = dict(min=list(request.min), max=list(request.max))
        response.success = True
        return response

    def stash_range_srv_cb(self, request, response):
        global lock
        ranges = [i[:-4] for i,j in self.get_parameters_by_prefix('color_range_list').items()  ]
        
        ranges = set(ranges) or self.__color_range_g
        
        with lock:
            ranges = self.__color_range_g
            ranges[request.color_name] = self.current_range.copy()
            self.set_parameters([rclpy.parameter.Parameter('color_range_list', value=ranges)])
            self.__color_range_g = ranges
        response.success = True
        return response
    
    def get_all_color_name_srv_cb(self, request, response):
        global lock
        ranges = [i[:-4] for i,j in self.get_parameters_by_prefix('color_range_list').items()]
        
        ranges = set(ranges) or self.__color_range_g
        #with lock:
        ranges = self.__color_range_g
        color_names = list(ranges.keys())
        response.color_names = color_names
        self.get_logger().info(str(response.color_names))  
        
        return response

    def heartbeat_srv_cb(self, request, response):
        global heartbeat_timer
        if isinstance(heartbeat_timer, Timer):
            heartbeat_timer.cancel()
        self.get_logger().debug("Heartbeat")
        if request.data:
            heartbeat_timer = Timer(5, self.exit_func, args=(None, None))
            heartbeat_timer.start()
        response.success = request.data
        return response

def main(args=None):
    rclpy.init(args=args)
    lab_config_manager = LabConfigManager()
    try:
        rclpy.spin(lab_config_manager)
    except KeyboardInterrupt:
        lab_config_manager.get_logger().info("Shutting down")
    finally:
        lab_config_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

 
