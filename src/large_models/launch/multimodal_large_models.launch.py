import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):

    ros_robot_controller_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/driver/ros_robot_controller/launch/ros_robot_controller.launch.py']),)
            
    usb_cam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/peripherals/launch/usb_cam.launch.py']),)
            
    puppy_control_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(['/home/ubuntu/ros2_ws/src/driver/puppy_control/launch/puppy_control.launch.py']),)
    
    vocal_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/vocal_detect.launch.py')),)
            
    function_call_node = Node(
            package='large_models', 
            executable='function_call', 
            #name='function_call', 
            output='screen',
        )
        
    visual_patrol_executable = Node(
            package='large_models', 
            executable='visual_patrol_node', 
            #name='function_call', 
            output='screen',
        )
        
    kick_ball_executable = Node(
            package='large_models', 
            executable='kick_ball_node', 
            #name='function_call', 
            output='screen',
        )

    agent_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/agent_process.launch.py')),
    )

    tts_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/tts_node.launch.py')),
    )

    return [
            ros_robot_controller_launch,
            usb_cam_launch,
            puppy_control_launch,         
            vocal_detect_launch,
            agent_process_launch,
            tts_node_launch,
            kick_ball_executable,
            visual_patrol_executable,
            function_call_node,
            
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()

