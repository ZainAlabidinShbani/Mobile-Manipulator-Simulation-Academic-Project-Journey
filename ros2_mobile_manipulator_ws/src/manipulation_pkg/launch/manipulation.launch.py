import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('manipulation_pkg')
    params = os.path.join(pkg_share, 'config', 'manipulation.yaml')
    return LaunchDescription([
        Node(package='manipulation_pkg', executable='arm_kinematics_node', parameters=[params], output='screen'),
        Node(package='manipulation_pkg', executable='trajectory_generator_node', parameters=[params], output='screen'),
        Node(package='manipulation_pkg', executable='arm_controller_node', parameters=[params], output='screen'),
    ])
