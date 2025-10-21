from distutils.core import setup

setup(
    name = 'arm_kinematics',
    version = '1.0',
    description = 'kinematics library',
    author = 'aiden',
    packages = ['arm_kinematics'],
    package_data = {
        'arm_kinematics': ['*.so'], 
    }
)
