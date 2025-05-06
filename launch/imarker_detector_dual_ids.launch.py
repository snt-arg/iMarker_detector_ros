import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imarker_detector_ros',
            executable='dualvision_ids.py',
            name='dualvision_ids_node',
            output='screen',
        ),
    ])