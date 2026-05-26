import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('perception_pkg')
    params = os.path.join(pkg_share, 'config', 'perception.yaml')
    return LaunchDescription([
        Node(package='perception_pkg', executable='camera_node', parameters=[params], output='screen'),
        Node(package='perception_pkg', executable='yolo_detector_node', parameters=[params], output='screen'),
        Node(package='perception_pkg', executable='grasp_pose_estimator_node', parameters=[params], output='screen'),
    ])
