import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mapping = {
        'simulation_pkg': 'simulation.launch.py',
        'perception_pkg': 'perception.launch.py',
        'navigation_pkg': 'navigation.launch.py',
        'manipulation_pkg': 'manipulation.launch.py',
        'task_manager_pkg': 'task_manager.launch.py',
    }
    launches = []
    for pkg, launch_name in mapping.items():
        pkg_share = get_package_share_directory(pkg)
        launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', launch_name))
            )
        )
    return LaunchDescription(launches)
