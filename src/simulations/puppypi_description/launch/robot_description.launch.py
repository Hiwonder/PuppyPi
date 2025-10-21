import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 定义 LaunchConfiguration 参数
    use_gui = LaunchConfiguration('use_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    namespace = LaunchConfiguration('namespace', default='')  # 定义 namespace
    use_namespace = LaunchConfiguration('use_namespace', default='false')

    # 声明 Launch 参数
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Use GUI for joint_state_publisher'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )
    frame_prefix_arg = DeclareLaunchArgument(
        'frame_prefix',
        default_value='',
        description='Frame prefix'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )
    use_namespace_arg = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Use namespace'
    )

    # 获取包的共享目录路径
    puppypi_description_package_path = get_package_share_directory('puppypi_description')
    urdf_path = os.path.join(puppypi_description_package_path, 'urdf', 'puppy.xacro')
    rviz_config_file = os.path.join(puppypi_description_package_path, 'rviz', 'view.rviz')

    # 生成机器人描述
    robot_description = Command(['xacro ', urdf_path])

    # 定义节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['/controller_manager/joint_states'],
            'rate': 20.0
        }]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
        remappings=[('/joint_states', 'joint_controller')]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': frame_prefix,
            'use_sim_time': use_sim_time
        }],
        arguments=[urdf_path],
        remappings=[('/tf', 'tf'),
                   ('/tf_static', 'tf_static')]
    )

    # 包含 RViz 的 Launch 文件
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(puppypi_description_package_path, 'launch', 'rviz.launch.py')
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'rviz_config': rviz_config_file
        }.items()
    )

    # 返回 LaunchDescription 对象
    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        frame_prefix_arg,
        use_sim_time_arg,
        namespace_arg,
        use_namespace_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_launch,
    ])

if __name__ == '__main__':
    # 创建 LaunchDescription 对象
    ld = generate_launch_description()

    # 创建并运行 LaunchService
    from launch import LaunchService
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
