import os
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    sim = LaunchConfiguration('sim', default='true').perform(context)
    use_joy = LaunchConfiguration('use_joy', default='true').perform(context)
    master_name = LaunchConfiguration('master_name', default='/').perform(context)
    robot_name = LaunchConfiguration('robot_name', default='/').perform(context)
    action_name = LaunchConfiguration('action_name', default='init').perform(context)

    sim_arg = DeclareLaunchArgument('sim', default_value=sim)
    master_name_arg = DeclareLaunchArgument('master_name', default_value=master_name)
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value=robot_name)
    use_joy_arg = DeclareLaunchArgument('use_joy', default_value=use_joy)
    action_name_arg = DeclareLaunchArgument('action_name', default_value=action_name)

    max_linear_sim = '0.7'
    max_linear = '0.2'
    max_angular_sim = '3.5'
    max_angular = '0.5'

    topic_prefix = '' if robot_name == '/' else '/%s'%robot_name
    frame_prefix = '' if robot_name == '/' else '%s/'%robot_name
    use_namespace = 'false' if robot_name == '/' else 'true'    
    namespace = '' if robot_name == '/' else robot_name
    use_sim_time = 'true' if sim == 'true' else 'false'

    map_frame = '{}map'.format(frame_prefix) if robot_name == master_name else '{}/map'.format(master_name)
    cmd_vel_topic = '{}/controller/cmd_vel'.format(topic_prefix)
    scan_raw = '{}/scan_raw'.format(topic_prefix)
    scan_topic = '{}/scan'.format(topic_prefix)
    odom_frame = '{}odom'.format(frame_prefix)
    base_frame = '{}base_footprint'.format(frame_prefix)
    lidar_frame = '{}lidar_frame'.format(frame_prefix)
    imu_frame = '{}imu_link'.format(frame_prefix)

    if compiled == 'True':
        puppypi_description_package_path = get_package_share_directory('puppypi_description')
        robot_controller_package_path = get_package_share_directory('ros_robot_controller')
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('puppy_control')
        slam_package_path = get_package_share_directory('slam')
    else:
        puppypi_description_package_path = '/home/ubuntu/ros2_ws/src/simulations/puppypi_description'
        robot_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/ros_robot_controller'
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/puppy_control'
        slam_package_path = '/home/ubuntu/ros2_ws/src/slam'
        
    if use_namespace == 'false':
        ekf_param = ReplaceString(source_file=os.path.join(slam_package_path, 'config/ekf.yaml'), replacements={'namespace/': ''})
    else:
        ekf_param = ReplaceString(source_file=os.path.join(slam_package_path, 'config/ekf.yaml'), replacements={"namespace/": (namespace, '/')})

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/puppy_control.launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'frame_prefix': frame_prefix,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'map_frame': map_frame,
            'imu_frame': imu_frame,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(puppypi_description_package_path, 'launch/robot_description.launch.py')
        ]),
        launch_arguments={
            'frame_prefix': frame_prefix,
            'use_gui': 'false',
            'use_rviz': 'false',
            'use_sim_time': 'false',
            'use_namespace': use_namespace,
            'namespace': namespace,
        }.items()
    )
    
    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_controller_package_path, 'launch/ros_robot_controller.launch.py')
        ]),
        launch_arguments={
            'imu_frame': imu_frame,
        }.items()
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/lidar.launch.py')),
        launch_arguments={
            'lidar_frame': lidar_frame,
            'scan_topic': scan_topic,
            'scan_raw': scan_raw,
        }.items(),
    )
    
    rf2o_laser_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_package_path, 'launch/include/rf2o_laser_odometry.launch.py')),
        launch_arguments={
            'lidar_frame': lidar_frame,
            'scan_topic': scan_topic,
            'scan_raw': '/scan',
        }.items(),
    )
    
    
    
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            #('odometry/rf2o', 'odom'),
            ('odometry/filtered', 'odom'),
            
        ],
    )
    
    odom_basefootprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '1.57', 'odom', 'base_footprint']
        )

    joystick_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')),
        launch_arguments={
            'max_linear': max_linear_sim if sim == 'true' else max_linear,  
            'max_angular': max_angular_sim if sim == 'true' else max_angular,
            'remap_cmd_vel': cmd_vel_topic
        }.items(),
        condition=IfCondition(use_joy)
    )

    return [sim_arg, 
            master_name_arg,
            robot_name_arg, 
            use_joy_arg,
            robot_description_launch,
            action_name_arg,
            controller_launch,
            robot_controller_launch,
            #odom_basefootprint,
            lidar_launch, 
            rf2o_laser_odometry,
            ekf_filter_node,
            joystick_control_launch

            ]

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
