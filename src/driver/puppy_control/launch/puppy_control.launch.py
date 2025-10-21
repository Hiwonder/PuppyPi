from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puppy_control',
            executable='puppy_control',
            name='puppy',
            output='screen',
            parameters=[
                {'joint_state_pub_topic': 'true'},   
                {'joint_state_controller_pub_topic': 'true'}, 
            ],
        ),
    ]
    )
