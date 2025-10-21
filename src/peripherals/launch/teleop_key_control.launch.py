import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # 声明参数(declare parameter)
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=os.environ['HOST'],
        description='Name of the robot'
    )

    # 设置ROS命名空间(set ROS namespace)
    push_namespace = PushRosNamespace(
        namespace=LaunchConfiguration('robot_name')
    )

    # teleop_key_control节点(teleop_key_control node)
    teleop_key_control_node = Node(
        package='peripherals',
        executable='teleop_key_control',
        name='teleop_key_control',
        output='screen',
        prefix='xterm -e'
    )

    # 创建启动描述(create start description)
    ld = LaunchDescription()

    # 添加参数声明和ROS命名空间设置(add parameter description and ROS namespace setting)
    ld.add_action(robot_name_arg)
    ld.add_action(push_namespace)

    # 添加节点(add node)
    ld.add_action(teleop_key_control_node)

    return ld

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
