import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    compiled = os.environ['need_compile']

    if compiled == 'True':
        puppypi_description_package_path = get_package_share_directory('puppypi_description')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        puppypi_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/puppypi_description'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(puppypi_description_package_path, 'launch/robot_description.launch.py')]),
        launch_arguments={
            'frame_prefix': '',
            'use_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': 'false',
            'use_namespace': 'false',
            'namespace': '',
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar_filter.launch.py')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(peripherals_package_path) + '/rviz/lidar_view.rviz'],
        output='screen',
    )

    return LaunchDescription([
        robot_description_launch,
        lidar_launch,
        rviz_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
