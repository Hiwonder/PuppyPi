from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example', 
            executable='hand_control_with_arm', 
            name='hand_control_with_arm_node', 
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                ['/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py']
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                ['/home/ubuntu/ros2_ws/src/peripherals/launch/web_video_server.launch.py']
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                ['/home/ubuntu/ros2_ws/src/peripherals/launch/usb_cam.launch.py']
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                ['/home/ubuntu/ros2_ws/src/driver/puppy_control/launch/puppy_control.launch.py']
            ),
        ),
    ])
