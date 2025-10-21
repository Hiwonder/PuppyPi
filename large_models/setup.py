import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'large_models'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vocal_detect = large_models.vocal_detect:main',
            'agent_process = large_models.agent_process:main',
            #'function_call = large_models.function_call:main',
            'tts_node = large_models.tts_node:main',
            'visual_patrol_node = large_models.visual_patrol_demo:main',
            'kick_ball_node = large_models.kick_ball_demo:main',
            'function_call = large_models.function_call:main',
            'vllm_watch_dogs = large_models.vllm_watch_dogs:main',
            'vllm_gender_identification = large_models.vllm_gender_identification:main',
            'llm_temperature_demo = large_models.llm_temperature_demo:main',
            'llm_tracking = large_models.llm_tracking:main', 
        ],
    },
)
