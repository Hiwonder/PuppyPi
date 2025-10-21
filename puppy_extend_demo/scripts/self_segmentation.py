#!/usr/bin/env python
# encoding: utf-8
import cv2
import queue
import rospy
import threading
import sys
import numpy as np
import mediapipe as mp
from std_srvs.srv import SetBool, Trigger, Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
sys.path.append('/home/ubuntu/software/puppypi_control/')
from servo_controller import setServoPulse
from action_group_control import runActionGroup, stopActionGroup
from HiwonderPuppy import HiwonderPuppy, PWMServoParams
class SegmentationNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.running = True
        self.mp_selfie_segmentation = mp.solutions.selfie_segmentation
        self.mp_drawing = mp.solutions.drawing_utils
        self.image_queue = queue.Queue(maxsize=2)
        self.BG_COLOR = (192, 192, 192)  # gray
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        rospy.loginfo('\033[1;32m%s\033[0m' % 'start')
        self.cli = rospy.ServiceProxy('/puppy_control/go_home', Empty)
        self.cli()
        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        try:
            # Convert the ROS image message to a CV2 image
            rgb_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            if self.image_queue.full():
                # If the queue is full, discard the oldest image
                self.image_queue.get()
            # Put the image into the queue
            self.image_queue.put(rgb_image)
        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")

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
                # To improve performance, optionally mark the image as not writeable to
                # pass by reference.
                image.flags.writeable = False
                results = selfie_segmentation.process(image)
                image.flags.writeable = True
                # Since the input image is in RGB, directly use it
                condition = np.stack((results.segmentation_mask,) * 3, axis=-1) > 0.1
                if bg_image is None:
                    bg_image = np.zeros(image.shape, dtype=np.uint8)
                    bg_image[:] = self.BG_COLOR
                output_image = np.where(condition, image, bg_image)
                # Display image (already in RGB)
                cv2.imshow('MediaPipe Selfie Segmentation', output_image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # Press q or esc to exit
                    break
            cv2.destroyAllWindows()

def main():
    node = SegmentationNode('self_segmentation')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()
        rospy.signal_shutdown('Shutdown')

if __name__ == "__main__":
    main()
