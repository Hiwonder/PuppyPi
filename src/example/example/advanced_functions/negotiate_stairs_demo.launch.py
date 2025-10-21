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
            executable='negotiate_stairs_demo', 
            name='negotiate_stairs_node', 
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                ['/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py']
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
        #Node(
           # package='example',
            #executable='/usr/bin/python3', 
            #name='negotiate_stairs_demo_python_node',
            #output='screen',
            #arguments=['/home/ubuntu/ros2_ws/src/example/example/advanced_functions/include/negotiate_stairs_demo.py']
        #),
    ])


