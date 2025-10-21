# import os
# from ament_index_python.packages import get_package_share_directory

# from launch import LaunchDescription, LaunchService
# from launch.actions import IncludeLaunchDescription, OpaqueFunction
# from launch.launch_description_sources import PythonLaunchDescriptionSource

# def launch_setup(context):
#     vocal_detect_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('large_models'), 'launch/vocal_detect.launch.py')),
#     )

#     agent_process_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('large_models'), 'launch/agent_process.launch.py')),
#     )

#     tts_node_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(get_package_share_directory('large_models'), 'launch/tts_node.launch.py')),
#     )

#     return [vocal_detect_launch,
#             agent_process_launch,
#             tts_node_launch,
#             ]

# def generate_launch_description():
#     return LaunchDescription([
#         OpaqueFunction(function = launch_setup)
#     ])

# if __name__ == '__main__':
#     # 创建一个LaunchDescription对象(create a LaunchDescription object)
#     ld = generate_launch_description()

#     ls = LaunchService()
#     ls.include_launch_description(ld)
#     ls.run()
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 检查环境变量 need_compile 决定 peripherals 包路径
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    # 引入 large_models 包中的子 Launch 文件
    vocal_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/vocal_detect.launch.py'))
    )

    agent_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/agent_process.launch.py'))
    )

    tts_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/tts_node.launch.py'))
    )

    # 启动 puppy_control 节点
    puppy_control_node = Node(
        package='puppy_control',
        executable='puppy_control',
        name='puppy',
        output='screen',
        parameters=[
            {'joint_state_pub_topic': 'true'},
            {'joint_state_controller_pub_topic': 'true'}
        ]
    )

    # 启动 USB 摄像头节点
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[os.path.join(peripherals_package_path, 'config', 'usb_cam_param_1.yaml')],
    )

    # 返回所有节点和子 Launch
    return LaunchDescription([
        vocal_detect_launch,
        agent_process_launch,
        tts_node_launch,
        puppy_control_node,
        camera_node
    ])
