import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    # 启动 vocal_detect 节点
    vocal_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/vocal_detect.launch.py'))
    )

    # 启动 agent_process 节点
    agent_process_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/agent_process.launch.py'))
    )

    # 启动 tts_node 节点
    tts_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('large_models'), 'launch/tts_node.launch.py'))
    )

    # 启动 llm_tracking.py 作为 Python 脚本
    llm_tracking_launch = Node(
        package='large_models',  # 使用 python 包启动
        executable='/usr/bin/python3',  # 执行 Python3
        name='llm_tracking_node',  # 节点名称
        output='screen',  # 输出到屏幕
        arguments=[
            os.path.join('/home/ubuntu/ros2_ws/src/large_models/large_models', 'llm_tracking.py')  # 路径指向 llm_tracking.py
        ]
    )

    # 启动 USB 摄像头节点
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[os.path.join(peripherals_package_path, 'config', 'usb_cam_param_1.yaml')]
    )

    # 返回所有节点和子 Launch
    return LaunchDescription([
        vocal_detect_launch,
        agent_process_launch,
        tts_node_launch,
        llm_tracking_launch,  # 启动 llm_tracking.py
        camera_node,
    ])
