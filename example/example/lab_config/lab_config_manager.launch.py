import os
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    kernel_erode = LaunchConfiguration('kernel_erode', default='5')
    kernel_dilate = LaunchConfiguration('kernel_dilate', default='5')
    
    kernel_erode_arg = DeclareLaunchArgument('kernel_erode',  default_value=kernel_erode)
    kernel_dilate_arg = DeclareLaunchArgument('kernel_dilate',  default_value=kernel_dilate)
    
    config_file_path = '/home/ubuntu/ros2_ws/src/example/example/config/lab_config.yaml'
      
    parameters = [
        {'config_file_path': config_file_path},
        {'kernel_erode': kernel_erode},
        {'kernel_dilate': kernel_dilate}
    ]  
        
    lab_config_node = Node(
        package='example',
        executable='lab_config',
        output='screen',
        parameters=parameters     
    )
    return LaunchDescription([kernel_erode_arg, kernel_dilate_arg, lab_config_node])
    
    
if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
