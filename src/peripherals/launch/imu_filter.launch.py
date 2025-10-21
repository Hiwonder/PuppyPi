import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService

def generate_launch_description():
    compiled = os.environ['need_compile']
    if compiled == 'True':
        calibration_package_path = get_package_share_directory('calibration')
    else:
        calibration_package_path = '/home/ubuntu/ros2_ws/src/calibration'
    imu_calib_node = Node(
        package='imu_calib',
        executable='apply_calib',
        name='imu_calib',
        output='screen',
        parameters=[{"calib_file": os.path.join(calibration_package_path, 'config/imu_calib.yaml')
                     }],
        remappings=[
            ('raw', 'ros_robot_controller/imu_raw'),
            ('corrected', 'imu_corrected')
            ]
        )


    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='imu_filter',
        output='screen',
        parameters=[
            {'use_mag': False,
            # 'gain_acc': 0.2,
            # 'bias_alpha': 0.2,
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'publish_debug_topics': False}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/imu/data_raw', 'imu_corrected'),
            ('imu/data', 'imu')
            ]
        )

    return LaunchDescription([
        imu_calib_node,
        imu_filter_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
