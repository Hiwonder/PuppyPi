from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    scan = LaunchConfiguration('scan', default='/scan_raw')
    odom = LaunchConfiguration('odom', default='/odom_rf2o')
    base_frame = LaunchConfiguration('base_frame', default='base_footprint') 
    odom_frame = LaunchConfiguration('odom_frame', default='odom')

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        namespace='', 
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        parameters=[
            {'laser_scan_topic': scan},
            {'odom_topic': odom},      
            {'publish_tf': False},
            {'base_frame_id': base_frame},
            {'odom_frame_id': odom_frame},
            {'init_pose_from_topic': ''},
            {'freq': 5.0},
            {'verbose': True},
            # arguments=['--ros-args', '--log-level', 'WARN'],
        ]
    )

    return LaunchDescription([
        rf2o_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
