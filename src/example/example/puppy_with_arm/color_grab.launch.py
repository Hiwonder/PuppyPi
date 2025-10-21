from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    commands = []
    for i in range(5):  
        commands.append(
            ExecuteProcess(
                cmd=['sudo', 'chmod', '666', f'/dev/gpiochip{i}'],
                output='screen'
            )
        )
    
    return LaunchDescription(commands)

