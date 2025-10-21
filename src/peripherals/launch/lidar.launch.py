import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    scan_topic = LaunchConfiguration('scan_topic', default='scan')

    # Declare arguments
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Topic name for lidar scan data'
    )
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value='lidar_frame',
        description='TF frame ID for the lidar'
    )

    # Path to the launch file and package directory
    peripherals_package_path = get_package_share_directory('peripherals')
    lidar_launch_path = os.path.join(peripherals_package_path, 'launch/include/ldlidar_LD19.launch.py')

    # Include LD19 launch file
    ld19_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        launch_arguments={
            'topic_name': LaunchConfiguration('scan_topic'),
            'frame_id': LaunchConfiguration('lidar_frame'),
            'scan_raw': scan_raw
        }.items()
    )
    
    # filter the data blocked by the arm
    laser_filters_config = os.path.join(peripherals_package_path, 'config/lidar_filters_config_ld19.yaml')
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        output='screen',
        parameters=[laser_filters_config],
        remappings=[('scan', scan_raw),
                    ('scan_filtered', scan_topic)]
    )
    

    return LaunchDescription([
        scan_topic_arg,
        lidar_frame_arg,
        ld19_launch,
        laser_filter_node,
        
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

