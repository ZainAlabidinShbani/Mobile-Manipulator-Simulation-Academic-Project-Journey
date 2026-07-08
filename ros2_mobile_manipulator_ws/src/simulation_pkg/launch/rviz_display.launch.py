"""
rviz_display.launch.py
======================
RViz-only visualisation for simulation_pkg (no Gazebo required).
Mirrors assembly_of_robot/launch/display.launch.py exactly, but
uses the xacro model from simulation_pkg.

Launches:
  1. robot_state_publisher     — TF tree from xacro URDF
  2. joint_state_publisher_gui — GUI sliders for all joints (arm + wheels)
  3. static_transform_publisher— odom → base_footprint
                                  (so RViz fixed-frame resolves without nav)
  4. rviz2                     — opens with default.rviz config

Usage:
  ros2 launch simulation_pkg rviz_display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_sim = get_package_share_directory('simulation_pkg')

    xacro_file  = os.path.join(pkg_sim, 'urdf', 'mobile_manipulator.urdf.xacro')
    rviz_config = os.path.join(pkg_sim, 'rviz', 'default.rviz')
    controllers_file = os.path.join(pkg_sim, 'config', 'controllers.yaml')

    # ── Launch argument: allow overriding the xacro path ──────────────
    declare_xacro_arg = DeclareLaunchArgument(
        name='xacro',
        default_value=xacro_file,
        description='Absolute path to the robot xacro/urdf file',
    )

    # ── robot_description: process xacro at launch time ───────────────
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('xacro'),
            ' controllers_yaml:=',
            controllers_file,
        ]),
        value_type=str,
    )
    robot_description = {'robot_description': robot_description_content}

    # 1. robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # 2. joint_state_publisher_gui — interactive sliders
    #    Needed so robot_state_publisher can compute the TF for
    #    revolute/continuous joints and show them in RViz.
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # 3. Static TF: odom → base_footprint
    #    Publishes a zero-offset static transform so that RViz can
    #    resolve the fixed frame (odom) without a running nav stack.
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        declare_xacro_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        static_tf_odom,
        rviz_node,
    ])
