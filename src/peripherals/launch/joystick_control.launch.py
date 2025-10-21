from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',  
                'deadzone': 0.1,  #
                'autorepeat_rate': 20.0  #
            }]
        ),
        Node(
            package='peripherals',
            executable='remote_control_joystick',
            name='remote_control_joystick',
            output='screen',
        )
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
