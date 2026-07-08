import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_pkg')
    params = os.path.join(pkg_share, 'config', 'navigation.yaml')
    return LaunchDescription([
        Node(package='navigation_pkg', executable='slam_node', parameters=[params], output='screen'),
        Node(package='navigation_pkg', executable='localization_node', parameters=[params], output='screen'),
        Node(package='navigation_pkg', executable='obstacle_avoidance_node', parameters=[params], output='screen'),
        Node(package='navigation_pkg', executable='nav2_bridge_node', parameters=[params], output='screen'),
    ])
