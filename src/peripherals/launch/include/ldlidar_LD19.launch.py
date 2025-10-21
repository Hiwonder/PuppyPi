from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    lidar_frame = LaunchConfiguration('lidar_frame', default='base_laser')
    scan_raw = LaunchConfiguration('scan_raw', default='scan_raw')
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value=lidar_frame)
    scan_raw_arg = DeclareLaunchArgument('scan_raw', default_value=scan_raw)

    ld19_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {
                'topic_name': 'scan',
                'product_name': 'LDLiDAR_LD19',
                'port_baudrate': 230400,
                'port_name': '/dev/ttyUSB0',
                'frame_id': lidar_frame,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0
            }
        ],
        remappings=[('scan', scan_raw)]
    )


    return LaunchDescription([
        lidar_frame_arg,
        scan_raw_arg,
        ld19_node,
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()


