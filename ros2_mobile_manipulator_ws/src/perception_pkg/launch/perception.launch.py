"""perception.launch.py

Phase 3 — Perception Launch
Starts yolo_detector_node and depth_localizer_node with parameters
loaded from config/perception_params.yaml.

Usage:
    ros2 launch perception_pkg perception.launch.py
    ros2 launch perception_pkg perception.launch.py model_path:=/abs/path/yolov8n.pt
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    pkg_share  = get_package_share_directory("perception_pkg")
    params_file = os.path.join(pkg_share, "config", "perception_params.yaml")

    declare_model = DeclareLaunchArgument(
        "model_path",
        default_value="yolov8n.pt",
        description="Path to YOLOv8 model (.pt or .onnx)",
    )
    declare_conf = DeclareLaunchArgument(
        "confidence_threshold",
        default_value="0.40",
        description="Minimum YOLO confidence score to publish",
    )
    declare_sim = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
    )

    yolo_node = Node(
        package="perception_pkg",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        output="screen",
        parameters=[
            params_file,
            {
                "model_path":           LaunchConfiguration("model_path"),
                "confidence_threshold": LaunchConfiguration("confidence_threshold"),
                "use_sim_time":         LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    depth_node = Node(
        package="perception_pkg",
        executable="depth_localizer_node",
        name="depth_localizer_node",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        declare_model,
        declare_conf,
        declare_sim,
        yolo_node,
        depth_node,
    ])
