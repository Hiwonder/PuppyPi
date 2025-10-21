from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/lab_config/launch/lab_config_manager.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/peripherals/launch/usb_cam.launch.py']),
        ),

        #IncludeLaunchDescription(
            #PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/peripherals/launch/lidar.launch.py']),
        #),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/navigation/launch/include/lidar_filter.launch.py']),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/driver/puppy_control/launch/puppy_control.launch.py']),
        ),
    ])

