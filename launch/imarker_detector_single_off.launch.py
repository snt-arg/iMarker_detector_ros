import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package share directory (needed for RViz config and YAML)
    from ament_index_python.packages import get_package_share_directory
    pkgDir = get_package_share_directory('imarker_detector_ros')
    print("[INFO] Package directory is: ", pkgDir)

    return LaunchDescription([
        # Declare launch argument
        DeclareLaunchArgument(
            'show_rviz',
            default_value='true',
            description='Whether to show RViz or not'
        ),

        # RViz node (conditionally launched)
        Node(
            condition=IfCondition(LaunchConfiguration('show_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=[
                '-d', os.path.join(pkgDir, 'rviz', 'rviz_single_vision.rviz')],
            output='screen'
        ),

        # Single vision node
        Node(
            package='imarker_detector_ros',
            executable='singlevision_off_rosbag.py',
            name='singlevision_off_rosbag_node',
            parameters=[os.path.join(pkgDir, 'config', 'configs.yaml')],
            output='screen',
        )
    ])