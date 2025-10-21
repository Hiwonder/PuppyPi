import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription

def generate_launch_description():
    compiled = os.environ['need_compile']
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value=frame_prefix)
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)
    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)

    if compiled == 'True':
        puppypi_description_package_path = get_package_share_directory('puppypi_description')
    else:
        puppypi_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/puppypi_description'
    urdf_path = os.path.join(puppypi_description_package_path, 'urdf/puppy.xacro')
    rviz_config_file = os.path.join(puppypi_description_package_path, 'rviz/view.rviz')

    robot_description = Command(['xacro ', urdf_path])
    
    # 动态TF转换
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )
    
    # 静态TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'frame_prefix': frame_prefix, 'use_sim_time': use_sim_time}],
        arguments=[urdf_path],
    )

    rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(puppypi_description_package_path, 'launch', 'rviz.launch.py')),
            launch_arguments={
                              'namespace': namespace,
                              'use_namespace': use_namespace,
                              'rviz_config': rviz_config_file}.items())

    # Timer action to delay rviz_node for 5 seconds
    delay_rviz_node = TimerAction(
        period=5.0,
        actions=[rviz_launch],
    )

    return LaunchDescription([
        frame_prefix_arg,
        use_sim_time_arg,
        namespace_arg,
        use_namespace_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        delay_rviz_node,
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
