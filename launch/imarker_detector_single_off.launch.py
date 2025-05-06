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
            executable='singlevision_off_rosbag.py',
            name='singlevision_off_rosbag_node',
            output='screen',
        ),
    ])


# def generate_launch_description():
#     # Get package share directory (needed for RViz config and YAML)
#     from ament_index_python.packages import get_package_share_directory
#     pkg_dir = get_package_share_directory('imarker_detector_ros')

#     print("Package directory:", pkg_dir)

#     return LaunchDescription([
#         # Declare launch argument
#         DeclareLaunchArgument(
#             'show_rviz',
#             default_value='true',
#             description='Whether to show RViz or not'
#         ),

#         # RViz node (conditionally launched)
#         Node(
#             condition=IfCondition(LaunchConfiguration('show_rviz')),
#             package='rviz2',
#             executable='rviz2',
#             name='rviz',
#             arguments=[
#                 '-d', os.path.join(pkg_dir, 'config', 'rviz_single_vision.rviz')],
#             output='screen'
#         ),

#         # Your main Python node
#         Node(
#             name='iMarker_algorithms',
#             package='imarker_detector_ros',
#             executable='singlevision_off_rosbag',
#             parameters=[os.path.join(pkg_dir, 'config', 'cfg_off.yaml')],
#             output='screen'
#         ),
#     ])
