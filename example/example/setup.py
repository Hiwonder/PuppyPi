import os
from glob import glob
from setuptools import setup

package_name = 'example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('example', '**/*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='2912150135@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visual_patrol_with_arm = example.puppy_with_arm.visual_patrol_with_arm:main',
            'color_grab = example.puppy_with_arm.include.color_grab:main',
            'color_detect_with_arm = example.puppy_with_arm.include.color_detect_with_arm:main',
            'kick_ball_demo = example.advanced_functions.include.kick_ball_demo:main',
            'visual_patrol_demo = example.advanced_functions.include.visual_patrol_demo:main',
            'negotiate_stairs_demo = example.advanced_functions.include.negotiate_stairs_demo:main',
            'color_sorting = example.color_sorting.color_sorting_node:main',
            'lab_config = example.lab_config.lab_config_manager:main',
            'hand_control_with_arm = example.puppy_with_arm.include.hand_control_with_arm:main',
            'hand_control = example.hand_control_pkg.hand_control_with_arm:main',
            'hand_gesture_control =example.hand_gesture_control.hand_gesture_control_node:main',
            'mediapipe_pose =example.mediapipe_pose.mediapipe_pose_node:main'

    
        ],
    },
)

