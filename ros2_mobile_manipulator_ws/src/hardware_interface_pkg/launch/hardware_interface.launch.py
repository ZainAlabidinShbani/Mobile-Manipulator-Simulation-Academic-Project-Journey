import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('hardware_interface_pkg')
    params = os.path.join(pkg_share, 'config', 'hardware_interface.yaml')
    return LaunchDescription([
        Node(package='hardware_interface_pkg', executable='hardware_bridge_node', parameters=[params], output='screen'),
    ])
