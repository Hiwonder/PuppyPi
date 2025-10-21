#!/usr/bin/env python3
# encoding: utf-8
# 颜色跟踪、姿态控制及RGB LED同步

import cv2
import math
import rospy
import threading
import numpy as np
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from interfaces.srv import SetPoint, SetPointRequest, SetPointResponse
from interfaces.srv import SetFloat64, SetFloat64Request, SetFloat64Response
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse 
from object_tracking_app.common import ColorPicker  
from puppy_control.msg import Velocity, Pose
from ros_robot_controller.msg import RGBState, RGBsState  
from common import PID, Misc  
from puppy_control.srv import SetRunActionName


class ObjectTracker:
    def __init__(self, color):
        self.target_lab, self.target_rgb = color
        self.last_color_circle = None
        self.image_proc_size = (320, 240)

    def __call__(self, image, result_image, threshold):
        img_h, img_w = image.shape[:2]
        image_resized = cv2.resize(image, self.image_proc_size)
        image_lab = cv2.cvtColor(image_resized, cv2.COLOR_RGB2LAB)
        image_blurred = cv2.GaussianBlur(image_lab, (5, 5), 5)

        min_color = [
            int(self.target_lab[0] - 50 * threshold * 2),
            int(self.target_lab[1] - 50 * threshold),
            int(self.target_lab[2] - 50 * threshold)
        ]
        max_color = [
            int(self.target_lab[0] + 50 * threshold * 2),
            int(self.target_lab[1] + 50 * threshold),
            int(self.target_lab[2] + 50 * threshold)
        ]

        mask = cv2.inRange(image_blurred, tuple(min_color), tuple(max_color))  # 二值化
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        contour_area = list(filter(lambda c: cv2.contourArea(c) > 100, contours))  # 剔除面积过小的轮廓
        circle = None

        if contour_area:
            if self.last_color_circle is None:
                contour = max(contour_area, key=cv2.contourArea)
                circle = cv2.minEnclosingCircle(contour)
            else:
                (last_x, last_y), _ = self.last_color_circle
                circles = [cv2.minEnclosingCircle(c) for c in contour_area]
                circle_dist = [
                    (c, math.sqrt((c[0][0] - last_x) ** 2 + (c[0][1] - last_y) ** 2))
                    for c in circles
                ]
                circle, dist = min(circle_dist, key=lambda item: item[1])
                if dist >= 100:
                    circle = None

            if circle:
                self.last_color_circle = circle
                (x, y), r = circle
                x = int(x / self.image_proc_size[0] * img_w)
                y = int(y / self.image_proc_size[1] * img_h)
                r = int(r / self.image_proc_size[0] * img_w)
                cv2.circle(result_image, (x, y), r, (self.target_rgb[0], self.target_rgb[1], self.target_rgb[2]), 2)

        return result_image


class ObjectTrackingNode:
    # 定义初始姿态Stand
    Stand = {
        'roll': math.radians(0),
        'pitch': math.radians(0),
        'yaw': 0.000,
        'height': -10,
        'x_shift': -0.5,
        'stance_x': 0,
        'stance_y': 0,
        'run_time': 300  
    }

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        rospy.loginfo("Object Tracking, Movement Control and RGB LED Node Initialized")
        
        self.color_picker = None
        self.tracker = None
        self.threshold = 0.2
        self.image_sub = None
        self.result_publisher = rospy.Publisher('~image_result', Image, queue_size=1)

        self.PuppyVelocityPub = rospy.Publisher('/puppy_control/velocity', Velocity, queue_size=1)
        self.PuppyPosePub = rospy.Publisher('/puppy_control/pose', Pose, queue_size=1)
        self.x_pid = PID.PID(P=0.001, I=0.0001, D=0.00005) 
        self.z_pid = PID.PID(P=0.003, I=0.001, D=0)
        self.PuppyPose = self.Stand.copy()
        self.__isRunning = False
        self.lock = threading.RLock()  

        self.x_dis = 0.5
        self.y_dis = 0.00167
        self.Z_DIS = 0.002
        self.z_dis = self.Z_DIS

        # RGB LED 控制相关
        self.rgb_pub = rospy.Publisher('/ros_robot_controller/set_rgb', RGBsState, queue_size=1)

        # 心跳包相关
        self.heartbeat_timer = None
        self.heartbeat_service = rospy.Service('~heartbeat', SetBool, self.heartbeat_srv_callback)

        # 服务初始化
        self.enter_srv = rospy.Service('~enter', Trigger, self.enter_srv_callback)
        self.exit_srv = rospy.Service('~exit', Trigger, self.exit_srv_callback)
        self.set_target_color_srv = rospy.Service('~set_target_color', SetPoint, self.set_target_color_srv_callback)
        self.get_target_color_srv = rospy.Service('~get_target_color', Trigger, self.get_target_color_srv_callback)
        self.set_threshold_srv = rospy.Service('~set_threshold', SetFloat64, self.set_threshold_srv_callback)
        self.set_running_srv = rospy.Service('~set_running', SetBool, self.set_running_srv_callback)

        self.runActionGroup_srv = rospy.ServiceProxy('/puppy_control/runActionGroup', SetRunActionName)
        rospy.loginfo("Waiting for /puppy_control/runActionGroup service...")
        try:
            rospy.wait_for_service('/puppy_control/runActionGroup', timeout=10)
            rospy.loginfo("/puppy_control/runActionGroup service is available")
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to connect to /puppy_control/runActionGroup service: {e}")
            self.runActionGroup_srv = None

        rospy.on_shutdown(self.shutdown_callback)

    def shutdown_callback(self):
        rospy.loginfo("Shutting down ObjectTrackingNode, resetting pose and RGB LEDs")
        self.cleanup()

    def cleanup(self):
        if self.image_sub is not None:
            self.image_sub.unregister()
            rospy.loginfo("Image subscriber unregistered")
        self.tracker = None
        self.color_picker = None
        self.stop_running()
        
        if self.runActionGroup_srv:
            try:
                rospy.loginfo("Calling runActionGroup_srv to reset pose to 'Stand.d6ac'")
                #self.runActionGroup_srv('Stand.d6ac', True)，#app发送了不用管
                rospy.loginfo("Successfully called runActionGroup_srv to reset pose")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to call runActionGroup_srv: {e}")
        else:
            rospy.logwarn("runActionGroup_srv is not available, pose not reset via action group")
        
        # 关闭 RGB LED
        self.set_rgb_leds(0, 0, 0)
        rospy.loginfo("Cleanup completed: Pose reset via action group and RGB LEDs turned off")
        
        with self.lock:
            if self.heartbeat_timer:
                self.heartbeat_timer.cancel()
                self.heartbeat_timer = None

    def enter_srv_callback(self, req):
        if self.image_sub is not None:
            self.image_sub.unregister()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.initMove(delay=False)
        self.reset_movement()  #
        return TriggerResponse(success=True, message="Entered tracking and movement mode")

    def exit_srv_callback(self, req):
        rospy.loginfo("Exiting Object Tracking and Movement Mode")
        self.cleanup()
        return TriggerResponse(success=True, message="Exited tracking and movement mode")

    def set_target_color_srv_callback(self, req: SetPointRequest):
        rospy.loginfo("Setting Target Color")
        x, y = req.data.x, req.data.y
        if x == -1 and y == -1:
            self.color_picker = None
            self.tracker = None
            # 关闭 RGB LED
            self.set_rgb_leds(0, 0, 0)
            rospy.loginfo("Target color cleared and RGB LEDs turned off")
        else:
            self.color_picker = ColorPicker(req.data, 20)
            self.tracker = None
            # 关闭 RGB LED，等待新的颜色被选取
            self.set_rgb_leds(0, 0, 0)
            rospy.loginfo(f"Color picker initialized with coordinates: ({x}, {y})")
        return SetPointResponse(success=True, message="Target color set")

    def get_target_color_srv_callback(self, req):
        rospy.loginfo("Getting Target Color")
        rsp = TriggerResponse(success=False, message="No target color set")
        if self.tracker and self.tracker.target_rgb:
            rsp.success = True
            rsp.message = "{},{},{}".format(
                int(self.tracker.target_rgb[0]),
                int(self.tracker.target_rgb[1]),
                int(self.tracker.target_rgb[2])
            )
            rospy.loginfo(f"Current target color: {rsp.message}")
        return rsp

    def set_threshold_srv_callback(self, req: SetFloat64Request):
        rospy.loginfo(f"Setting Threshold to {req.data}")
        self.threshold = req.data
        rospy.loginfo(f"Threshold set to {self.threshold}")
        return SetFloat64Response(success=True, message="Threshold set")

    def set_running_srv_callback(self, req: SetBoolRequest):
        if req.data:
            self.start_running()
            rospy.loginfo("Movement control started via service call")
            return SetBoolResponse(success=True, message="Movement control started")
        else:
            self.stop_running()
            rospy.loginfo("Movement control stopped via service call")
            return SetBoolResponse(success=True, message="Movement control stopped")

    def start_running(self):
        """开始运动控制"""
        with self.lock:
            if not self.__isRunning:
                self.__isRunning = True
                rospy.loginfo("Movement control started")

    def stop_running(self):
        """停止运动控制并重置姿态，关闭 RGB LED"""
        with self.lock:
            if self.__isRunning:
                self.__isRunning = False
                rospy.loginfo("Movement control stopped")
                self.reset_pose()
                # 关闭 RGB LED
                self.set_rgb_leds(0, 0, 0)
                rospy.loginfo("Pose reset and RGB LEDs turned off after stopping movement control")

    def reset_pose(self):
        """重置姿态到初始状态并发布Pose消息"""
        with self.lock:
            self.PuppyPose = self.Stand.copy()
            pose_msg = Pose(
                stance_x=self.PuppyPose['stance_x'],
                stance_y=self.PuppyPose['stance_y'],
                x_shift=self.PuppyPose['x_shift'],
                height=self.PuppyPose['height'],
                roll=self.PuppyPose['roll'],
                pitch=self.PuppyPose['pitch'],
                yaw=self.PuppyPose['yaw'],
                run_time=self.PuppyPose['run_time']  
            )
            self.PuppyPosePub.publish(pose_msg)
            rospy.loginfo(f"Pose reset to initial state: {self.PuppyPose}")

    def reset_movement(self):
        """重置运动相关的PID控制器、速度变量和姿态"""
        with self.lock:
            rospy.loginfo("Resetting movement control variables, PID controllers, and velocities")
            # 重置PID控制器
            self.x_pid.clear()
            self.z_pid.clear()
            # 重置速度变量
            self.x_dis = 0.05
            self.y_dis = 0.00167
            self.Z_DIS = 0.002
            self.z_dis = self.Z_DIS
            rospy.loginfo(f"Speed variables reset to x_dis={self.x_dis}, y_dis={self.y_dis}, Z_DIS={self.Z_DIS}, z_dis={self.z_dis}")
            self.reset_pose()
            # 关闭 RGB LED
            self.set_rgb_leds(0, 0, 0)
            rospy.loginfo("Movement control variables, PID controllers, and velocities have been reset")

    def initMove(self, delay=False):
        """初始化运动到初始位置"""
        if self.runActionGroup_srv:
            try:
                rospy.loginfo("Initializing movement to initial position (Stand.d6ac)")
                # 调用动作组，#app发送了不用管
                self.runActionGroup_srv('Stand.d6ac', True)
                rospy.loginfo("Successfully called /puppy_control/runActionGroup to initialize movement")
            except rospy.ServiceException as e:
                rospy.logerr(f"Failed to call /puppy_control/runActionGroup service: {e}")
        else:
            rospy.logwarn("/puppy_control/runActionGroup service proxy is not initialized")
        
        if delay:
            rospy.sleep(2)  # 根据需要调整延迟时间

    def image_callback(self, ros_image: Image):
        # 将ROS图像消息转换为OpenCV图像
        try:
            rgb_image = np.frombuffer(ros_image.data, dtype=np.uint8).reshape((ros_image.height, ros_image.width, 3))
        except ValueError as e:
            rospy.logerr(f"Image data reshaping error: {e}")
            return

        result_image = np.copy(rgb_image)

        if self.color_picker:
            target_color, result_image = self.color_picker(rgb_image, result_image)
            if target_color:
                rospy.loginfo("Target color picked")
                self.color_picker = None
                self.tracker = ObjectTracker(target_color)
        elif self.tracker and self.__isRunning:
            try:
                result_image = self.tracker(rgb_image, result_image, self.threshold)
                self.control_movement()
            except Exception as e:
                rospy.logerr(f"Error in tracking: {e}")

        result_msg = Image()
        result_msg.header = ros_image.header
        result_msg.height = result_image.shape[0]
        result_msg.width = result_image.shape[1]
        result_msg.encoding = ros_image.encoding
        result_msg.is_bigendian = ros_image.is_bigendian
        result_msg.step = result_image.shape[1] * 3
        result_msg.data = result_image.tobytes()
        self.result_publisher.publish(result_msg)

    def control_movement(self):
        with self.lock:
            if self.tracker.last_color_circle:
                (x, y), r = self.tracker.last_color_circle
                img_w, img_h = self.tracker.image_proc_size

                original_img_w = rospy.get_param('~original_image_width', 640)  
                original_img_h = rospy.get_param('~original_image_height', 480)  

                # 将坐标从处理后的图像尺寸映射回原始图像尺寸
                x_original = int(x / img_w * original_img_w)
                y_original = int(y / img_h * original_img_h)
                r_original = int(r / img_w * original_img_w)

                # PID 控制
                self.x_pid.SetPoint = original_img_w / 2.0
                self.x_pid.update(x_original)
                x_dis = self.x_pid.output
                x_dis = np.clip(x_dis, -0.25, 0.25) 

                self.z_pid.SetPoint = original_img_h / 2.0
                self.z_pid.update(y_original)
                z_dis = self.z_pid.output
                z_dis = np.clip(z_dis, -0.2, 0.3)  

                # 更新速度变量
                self.x_dis = x_dis
                # 根据需要更新 y_dis
                # 根据目标面积调整 y_dis
                # self.y_dis = some_logic_based_on_area
                self.z_dis = z_dis

                # 更新姿态
                self.PuppyPose['roll'] = self.x_dis
                self.PuppyPose['pitch'] = self.z_dis

                # 发布姿态
                pose_msg = Pose(
                    stance_x=self.PuppyPose['stance_x'],
                    stance_y=self.PuppyPose['stance_y'],
                    x_shift=self.PuppyPose['x_shift'],
                    height=self.PuppyPose['height'],
                    roll=self.PuppyPose['roll'],
                    pitch=self.PuppyPose['pitch'],
                    yaw=self.PuppyPose['yaw'],
                    run_time=self.PuppyPose['run_time']  
                )
                self.PuppyPosePub.publish(pose_msg)
                rospy.logdebug(f"Published Pose: {pose_msg}")

                # 发布速度消息
                velocity_msg = Velocity(
                    x=self.x_dis,
                    y=self.y_dis,
                    yaw_rate=self.z_dis
                )
                self.PuppyVelocityPub.publish(velocity_msg)
                rospy.logdebug(f"Published Velocity: {velocity_msg}")

                # 设置 RGB LED 颜色为目标颜色
                self.set_rgb_leds(*self.tracker.target_rgb)
            else:
                # 如果没有检测到目标，关闭 RGB LED
                self.set_rgb_leds(0, 0, 0)
                rospy.loginfo("No target detected, RGB LEDs turned off")

    def set_rgb_leds(self, r, g, b):
        """设置 RGB LED 的颜色"""
        rgb_msg = RGBsState()
        led1 = RGBState(id=1, r=r, g=g, b=b)
        led2 = RGBState(id=2, r=r, g=g, b=b)
        rgb_msg.data = [led1, led2]
        self.rgb_pub.publish(rgb_msg)
        rospy.logdebug(f"Set RGB LEDs to R:{r} G:{g} B:{b}")

    # 实现心跳包逻辑
    def heartbeat_srv_callback(self, req: SetBoolRequest):
        """心跳服务回调函数"""
        with self.lock:
            if self.heartbeat_timer:
                self.heartbeat_timer.cancel()
                rospy.logdebug("Existing heartbeat timer canceled")

            if req.data:
                self.heartbeat_timer = threading.Timer(5, self.call_exit_service)
                self.heartbeat_timer.start()
                rospy.logdebug("Heartbeat timer started/reset")
        
        rsp = SetBoolResponse()
        rsp.success = req.data
        rsp.message = "Heartbeat received" if req.data else "Heartbeat stopped"
        return rsp

    def call_exit_service(self):
        """调用退出服务以停止节点"""
        rospy.logwarn("Heartbeat timeout! Calling exit service to stop the node.")
        try:
            exit_srv = rospy.ServiceProxy('~exit', Trigger)
            exit_srv(TriggerRequest())
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to call exit service: {e}")

    def run_debug(self):
        """可选的调试方法，用于以调试模式运行节点。"""
        rospy.sleep(0.2)
        self.enter_srv_callback(TriggerRequest())

        msg = SetPointRequest()
        msg.data.x = 100
        msg.data.y = 100
        self.set_target_color_srv_callback(msg)
        self.start_running()


if __name__ == "__main__":
    node = None
    try:
        node = ObjectTrackingNode('object_tracking')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Exception in ObjectTrackingNode: {e}")  
    finally:
        if node is not None:
            node.cleanup()
