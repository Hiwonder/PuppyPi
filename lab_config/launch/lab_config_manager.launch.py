
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    kernel_erode = LaunchConfiguration('kernel_erode', default='5')
    kernel_dilate = LaunchConfiguration('kernel_dilate', default='5')
    
    kernel_erode_arg = DeclareLaunchArgument('kernel_erode',  default_value=kernel_erode)
    kernel_dilate_arg = DeclareLaunchArgument('kernel_dilate',  default_value=kernel_dilate)
    driver_package_path = '/home/ubuntu/ros2_ws/src/lab_config'

      
    parameters = [
        os.path.join(driver_package_path,'config/lab_config.yaml'),
        {'config_file_path': os.path.join(driver_package_path,'config/lab_config.yaml')},
        {'kernel_erode': kernel_erode},
        {'kernel_dilate': kernel_dilate}
    ]  
        
    lab_config_node = Node(
        package='lab_config',
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
