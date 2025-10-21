from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example', 
            executable='kick_ball_demo', 
            name='kick_ball_demo_node', 
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
        # 使用 Python 3 的完整路径启动脚本
        #Node(
            #package='example',
            #executable='/usr/bin/python3',  # 指定 Python 3 的完整路径
            #name='kick_ball_demo_python_node',
            #output='screen',
            #arguments=['/home/ubuntu/ros2_ws/src/example/example/advanced_functions/include/kick_ball_demo.py']
        #),
    ])
