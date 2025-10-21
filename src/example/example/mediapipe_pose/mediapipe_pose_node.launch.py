import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)
    
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        robot_controller_package_path = get_package_share_directory('ros_robot_controller')
        controller_package_path = get_package_share_directory('puppy_control')
        example_package_path = get_package_share_directory('example')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        robot_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/ros_robot_controller'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/puppy_control'
        example_package_path = '/home/ubuntu/ros2_ws/src/example'
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/usb_cam.launch.py')),
    )
    
    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(robot_controller_package_path, 'launch/ros_robot_controller.launch.py')
        ]),
        launch_arguments={
            'imu_frame': imu_frame,
        }.items()
    )
    
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_package_path, 'launch/puppy_control.launch.py')),
    )
    

    mediapipe_pose_node = Node(
        package='example',
        executable='mediapipe_pose',
        output='screen',
    )

    return [depth_camera_launch,
            robot_controller_launch,
            controller_launch,
            mediapipe_pose_node,
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
