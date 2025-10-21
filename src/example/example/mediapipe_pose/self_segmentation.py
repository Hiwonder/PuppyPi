#!/usr/bin/env python3
# encoding: utf-8
import cv2
import sys
from sdk import fps
import queue
import rclpy
import threading
import numpy as np
import mediapipe as mp
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger, Empty
from puppy_control_msgs.srv import SetRunActionName
from cv_bridge import CvBridge  # Import CvBridge for conversion
sys.path.append('/home/ubuntu/software/puppypi_control/') 
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup

class SegmentationNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.running = True
        self.mp_selfie_segmentation = mp.solutions.selfie_segmentation
        self.mp_drawing = mp.solutions.drawing_utils
        self.fps = fps.FPS()
        self.image_queue = queue.Queue(maxsize=2)
        self.BG_COLOR = (192, 192, 192)  # gray
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.cli = self.create_client(Empty,'/puppy_control/go_home')
        self.cli.call_async(Empty.Request())
        
        self.bridge = CvBridge()  # Initialize CvBridge
        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting ROS image to OpenCV: {e}")
            return
        
        if self.image_queue.full():
            self.image_queue.get()
        self.image_queue.put(rgb_image)

    def main(self):
        with self.mp_selfie_segmentation.SelfieSegmentation(model_selection=1) as selfie_segmentation:
            bg_image = None
            while self.running:
                try:
                    image = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    if not self.running:
                        break
                    else:
                        continue
                
                # To improve performance, optionally mark the image as not writeable to pass by reference.
                image.flags.writeable = False
                results = selfie_segmentation.process(image)
                image.flags.writeable = True
                
                # image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                
                # Draw selfie segmentation on the background image.
                condition = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.1
                
                # 如果没有背景图像，则创建一个灰色背景
                if bg_image is None:
                    bg_image = np.zeros(image.shape, dtype=np.uint8)
                    bg_image[:] = self.BG_COLOR
                
                # 根据分割结果合成输出图像
                output_image = np.where(condition, image, bg_image)
                
                self.fps.update()
                result_image = self.fps.show_fps(output_image)
                
                cv2.imshow('MediaPipe Selfie Segmentation', result_image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # Press q or esc to exit
                    break
            
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main():
    node = SegmentationNode('self_segmentation')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        print('shutdown')
    finally:
        print('shutdown finish')

if __name__ == "__main__":
    main()
