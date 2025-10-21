#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['visual_patrol_app'],
    package_dir={'': 'src'},
)

setup(**d)

