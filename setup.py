#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['iMarker_sensors', 'iMarker_algorithms'],
    package_dir={'iMarker_sensors': 'src/iMarker_sensors',  # Packages within the ROS workspace
                 'iMarker_algorithms': 'src/iMarker_algorithms'  # Location of the packages
                 }
)

setup(**d)
