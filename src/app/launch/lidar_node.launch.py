from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define the paths to the launch files
    lidar_launch_path = os.path.join(get_package_share_directory('peripherals'), 'launch', 'lidar.launch.py')
    puppy_control_launch_path = os.path.join('/home/ubuntu/ros2_ws/src/driver/puppy_control/launch', 'puppy_control.launch.py')

    return LaunchDescription([
        # Include the lidar launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_launch_path)
        ),

        # Node for lidar application
        Node(
            package='app',
            executable='lidar',
            name='lidar_app',
            output='screen'
        ),
        
        # Include the puppy control launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(puppy_control_launch_path)
        ),
    ])

