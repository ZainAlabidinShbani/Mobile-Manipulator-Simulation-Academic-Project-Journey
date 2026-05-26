import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('perception_pkg')
    params_file = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.40'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='perception_pkg',
            executable='yolo_detector_node',
            name='yolo_detector_node',
            output='screen',
            parameters=[params_file, {
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='perception_pkg',
            executable='depth_localizer_node',
            name='depth_localizer_node',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
