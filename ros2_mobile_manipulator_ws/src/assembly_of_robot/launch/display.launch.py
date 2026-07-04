"""
display.launch.py — RViz2 visualisation for assembly_of_robot
==============================================================
Launches:
  1. robot_state_publisher  — broadcasts URDF TF tree
  2. joint_state_publisher_gui  — GUI sliders to move joints
  3. static_transform_publisher  — publishes odom -> base_link
     so RViz fixed frame resolves without a nav stack
  4. rviz2  — opens with a pre-configured .rviz file

Usage:
  ros2 launch assembly_of_robot display.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('assembly_of_robot')

    # Path to URDF file inside the installed package share
    urdf_path = os.path.join(pkg_share, 'urdf', 'assembly_of_robot.urdf')

    # Path to RViz config (falls back gracefully if not present)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # Allow overriding the URDF path from command line
    declare_urdf_arg = DeclareLaunchArgument(
        name='urdf',
        default_value=urdf_path,
        description='Absolute path to the robot URDF file'
    )

    # Read URDF content and pass it as robot_description parameter
    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf')]),
        value_type=str
    )

    # 1. robot_state_publisher — publishes all TF frames from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. joint_state_publisher_gui — interactive sliders for joints
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # 3. Static TF: odom -> base_link (required for RViz fixed frame)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        declare_urdf_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        static_tf_node,
        rviz_node,
    ])
