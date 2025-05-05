#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    name='imarker_detector_ros',
    version='1.0.0',
    author='Ali Tourani',
    author_email='ali.tourani@uni.lu',
    description='A ROS-based software to detect iMarkers',
    long_description='A ROS-wrapped package providing a functionalities to detect iMarkers.',
    long_description_content_type='text/plain',
    url='https://github.com/snt-arg/iMarker_detector_ros',
    packages=['iMarker_sensors', 'iMarker_algorithms'],
    package_dir={'iMarker_sensors': 'src/iMarker_sensors',  # Packages within the ROS workspace
                 'iMarker_algorithms': 'src/iMarker_algorithms'  # Location of the packages
                 }
)

setup(**d)
