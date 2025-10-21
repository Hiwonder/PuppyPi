import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puppy_control'

setup(
    name=package_name,
    version='0.0.1',  
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  
        (os.path.join('lib', package_name), glob('scripts/*.py')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Puppy control package for controlling a robot.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puppy_control = puppy_control.puppy:main',  
        ],
    },
)

