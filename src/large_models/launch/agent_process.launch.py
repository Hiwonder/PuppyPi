from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction, DeclareLaunchArgument

def launch_setup(context):
    camera_topic = LaunchConfiguration('camera_topic', default='image_raw')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value=camera_topic)

    agent_process_node = Node(
        package='large_models',
        executable='agent_process',
        output='screen',
        parameters=[{"camera_topic": camera_topic}],
    )

    return [
            camera_topic_arg,
            agent_process_node
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
