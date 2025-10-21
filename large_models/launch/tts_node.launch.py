from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import OpaqueFunction 

def launch_setup(context):
    tts_node = Node(
        package='large_models',
        executable='tts_node',
        output='screen',
    )

    return [
            tts_node
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
