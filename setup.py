#!/usr/bin/env python3

from glob import glob
from setuptools import setup

package_name = 'imarker_detector_ros'

setup(
    name=package_name,
    version='1.0.0',
    author='Ali Tourani',
    author_email='ali.tourani@uni.lu',
    description='A ROS 2 package to detect iMarkers',
    long_description='A ROS 2-wrapped package providing functionalities to detect iMarkers.',
    long_description_content_type='text/plain',
    url='https://github.com/snt-arg/iMarker_detector_ros',
    packages=[
        'iMarker_sensors',
        'iMarker_algorithms',
        'marker_detector',
        'utils'
    ],
    package_dir={
        'iMarker_sensors': 'src/iMarker_sensors',
        'iMarker_algorithms': 'src/iMarker_algorithms',
        'marker_detector': 'src/marker_detector',
        'utils': 'src/utils'
    },
    data_files=[
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/config', glob('config/*.rviz')),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/src',
         glob('src/notFound.png')),  # Include image
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'singlevision_off_rosbag = singlevision_off_rosbag:main'
        ],
    },
)
