import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('task_manager_pkg')
    params = os.path.join(pkg_share, 'config', 'task_manager.yaml')
    return LaunchDescription([
        Node(package='task_manager_pkg', executable='fsm_task_manager_node', parameters=[params], output='screen'),
    ])
