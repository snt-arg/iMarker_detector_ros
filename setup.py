#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['csr_sensors', 'csr_detector'],
    package_dir={'csr_sensors': 'src/csr_sensors',  # Packages within the ROS workspace
                 'csr_detector': 'src/csr_detector'  # Location of the packages
                 }
)

setup(**d)
