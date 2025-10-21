import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        navigation_package_path = get_package_share_directory('navigation')
    else:
        navigation_package_path = '/home/ubuntu/ros2_ws/src/navigation'

    # Create the launch configuration variables
    rtabmap = LaunchConfiguration('rtabmap')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace').perform(context)
    use_teb = LaunchConfiguration('use_teb', default='true').perform(context)
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    declare_rtabmap_cmd = DeclareLaunchArgument(
        'rtabmap',
        default_value='false',
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(navigation_package_path, 'maps', 'map_01.yaml'),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(navigation_package_path, 'config/nav2_params.yaml')
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_use_teb_cmd = DeclareLaunchArgument(
        'use_teb', default_value='true',
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[params_file, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/localization.launch.py')),
            condition=UnlessCondition(rtabmap),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'params_file': params_file,
                              'use_sim_time': use_sim_time,
                              'use_namespace': use_namespace,
                              'autostart': autostart}.items()),
            

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation_package_path, 'launch/include/navigation_base.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_namespace': use_namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_teb': use_teb}.items()),
    ])

    return [declare_rtabmap_cmd,
            stdout_linebuf_envvar, 
            declare_namespace_cmd, 
            declare_use_namespace_cmd, 
            declare_map_yaml_cmd, 
            declare_use_sim_time_cmd, 
            declare_autostart_cmd, 
            declare_params_file_cmd, 
            declare_log_level_cmd,
            declare_use_teb_cmd,
            bringup_cmd_group]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
