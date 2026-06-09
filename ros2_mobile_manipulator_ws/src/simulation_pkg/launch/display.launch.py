"""Alias launcher for rviz_display.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_sim = get_package_share_directory('simulation_pkg')
    rviz_display_launch = os.path.join(pkg_sim, 'launch', 'rviz_display.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_display_launch),
        )
    ])
