import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    compiled = os.environ['need_compile']
    camera_type = os.environ['DEPTH_CAMERA_TYPE']
    depth_camera_name_arg = DeclareLaunchArgument(
        'depth_camera_name',
        default_value='depth_cam'
    )

    tf_prefix_arg = DeclareLaunchArgument(
        'tf_prefix',
        default_value=''
    )
    color_height_arg = DeclareLaunchArgument(
        'color_height',
        default_value='480'
    )

    app_arg = DeclareLaunchArgument(
        'app',
        default_value='false'
    )

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    if camera_type == 'Dabai':
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/include/dabai_dcw.launch.py')),
            launch_arguments={
                'camera_name': LaunchConfiguration('depth_camera_name'),
                'tf_prefix': LaunchConfiguration('tf_prefix'),
                'color_height': LaunchConfiguration('color_height'),
            }.items())
    else:
        camera_launch = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(peripherals_package_path, 'launch/usb_cam.launch.py'))
                ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="depth_cam_base_link",
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--qx', '-1.5707963267948966',
                    '--qy', '0',
                    '--qz', '-1.5707963267948966',
                    '--qw', '0',
                    '--frame-id', 'depth_cam_link',
                    '--child-frame-id', 'depth_cam_color_frame'
                ]
            ),
        ])

    return LaunchDescription([
        depth_camera_name_arg,
        color_height_arg,
        tf_prefix_arg,
        app_arg,
        camera_launch
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
