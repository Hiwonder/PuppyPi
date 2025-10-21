from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
   
    
    publish_point_node = Node(
        package = 'navigation',
        executable = 'publish_point',
        output='screen',    
    )

    return LaunchDescription([
        publish_point_node
    ])

if __name__ == '__main__':
    
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
