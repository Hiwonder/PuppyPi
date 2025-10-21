from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

def generate_launch_description():
    # Define paths to external launch files
    ros_robot_controller_launch_path = '/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py'
    usb_cam_launch_path = '/home/ubuntu/ros2_ws/src/peripherals/launch/usb_cam.launch.py'
    puppy_control_launch_path = '/home/ubuntu/ros2_ws/src/driver/puppy_control/launch/puppy_control.launch.py'

    return LaunchDescription([
        # Define the color detect with arm node
        Node(
            package='example',
            executable='color_detect_with_arm',
            name='color_detect_with_arm_node',
            output='screen'
        ),

        # Include ros_robot_controller launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_robot_controller_launch_path)
        ),

        # Include usb_cam launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(usb_cam_launch_path)
        ),

        # Include puppy_control launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(puppy_control_launch_path)
        ),
    ])

