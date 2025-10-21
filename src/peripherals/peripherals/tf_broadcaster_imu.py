#!/usr/bin/env python3
# encoding: utf-8 
import rclpy
import tf2_ros
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

def qua2rot(x, y, z, w):
    rot_matrix = np.matrix(
        [[1.0 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (w * y + x * z)],
        [2 * (x * y + w * z), 1.0 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1.0 - 2 * (x * x + y * y)]])

    return rot_matrix

def rot2qua(M):
    Qxx, Qyx, Qzx, Qxy, Qyy, Qzy, Qxz, Qyz, Qzz = M.flat
    K = np.array([
        [Qxx - Qyy - Qzz, 0,               0,               0              ],
        [Qyx + Qxy,       Qyy - Qxx - Qzz, 0,               0              ],
        [Qzx + Qxz,       Qzy + Qyz,       Qzz - Qxx - Qyy, 0              ],
        [Qyz - Qzy,       Qzx - Qxz,       Qxy - Qyx,       Qxx + Qyy + Qzz]]
        ) / 3.0
    vals, vecs = np.linalg.eigh(K)
    q = vecs[[3, 0, 1, 2], np.argmax(vals)]
    if q[0] < 0:
        q *= -1
    return [q[1], q[2], q[3], q[0]]

class IMUTransformBroadcaster(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        # 声明参数(declare parameter)
        self.declare_parameter('imu_link', 'imu_link')
        self.declare_parameter('imu_frame', 'imu_frame')
        self.declare_parameter('imu_topic', 'imu')

        self.target_frame = self.get_parameter('imu_link').value
        self.imu_frame = self.get_parameter('imu_frame').value
        self.imu_topic = self.get_parameter('imu_topic').value
        
        self.trans = TransformStamped()
        self.imu_broadcaster = tf2_ros.TransformBroadcaster(self)  # 定义TF变换广播者(define TF transform broadcaster)
        self.create_subscription(Imu, self.imu_topic, self.handle_imu_pose, 1)
        self.clock = self.get_clock()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def handle_imu_pose(self, msg):
        self.trans.header.stamp = self.clock.now().to_msg()
        self.trans.header.frame_id = self.target_frame
        self.trans.child_frame_id = self.imu_frame
        self.trans.transform.translation.x = 0.0
        self.trans.transform.translation.y = 0.0
        self.trans.transform.translation.z = 0.0
        rot = qua2rot(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        qua = rot2qua(rot.I)
        
        self.trans.transform.rotation.x = qua[0]
        self.trans.transform.rotation.y = qua[1]
        self.trans.transform.rotation.z = qua[2]
        self.trans.transform.rotation.w = qua[3]

        self.imu_broadcaster.sendTransform(self.trans)

def main():
    node = IMUTransformBroadcaster('tf_broadcaster_imu')
    rclpy.spin(node)  # 循环等待ROS2退出(loop and wait for ROS2 to exit)

if __name__ == "__main__":
    main()
