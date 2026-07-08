#!/usr/bin/env python3
"""
nav2.launch.py

Launches the complete navigation stack:
  - SLAM Toolbox  (online async mapping)
  - Nav2 bringup  (planner + controller + behaviours)
  - navigation_node (goal management)

Usage:
  ros2 launch navigation_pkg nav2.launch.py
  ros2 launch navigation_pkg nav2.launch.py slam:=false map:=/path/to/map.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package share paths ──────────────────────────────────────────────────────
    nav_pkg  = FindPackageShare('navigation_pkg')
    nav2_pkg = FindPackageShare('nav2_bringup')
    slam_pkg = FindPackageShare('slam_toolbox')

    nav2_params_file = PathJoinSubstitution([nav_pkg, 'config', 'nav2_params.yaml'])
    slam_params_file = PathJoinSubstitution([nav_pkg, 'config', 'slam_toolbox_params.yaml'])

    # ── Launch arguments ──────────────────────────────────────────────────────
    declare_slam = DeclareLaunchArgument(
        'slam', default_value='true',
        description='true = SLAM mapping, false = use saved map'
    )
    declare_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to saved map YAML (slam:=false only)'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock'
    )
    declare_auto_nav = DeclareLaunchArgument(
        'auto_navigate_on_detection', default_value='true',
        description='Auto-navigate when YOLO detects target object'
    )
    declare_target = DeclareLaunchArgument(
        'target_class', default_value='cube',
        description='YOLO class that triggers navigation'
    )

    slam         = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    auto_nav     = LaunchConfiguration('auto_navigate_on_detection')
    target_class = LaunchConfiguration('target_class')

    # ── SLAM Toolbox (online async) ──────────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([slam_pkg, 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items(),
        condition=IfCondition(slam)
    )

    # ── Nav2 bringup ─────────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_pkg, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
            'map':          map_yaml,
        }.items()
    )

    # ── Navigation node ─────────────────────────────────────────────────────────
    navigation_node = Node(
        package='navigation_pkg',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time':               use_sim_time,
            'auto_navigate_on_detection': auto_nav,
            'target_class':               target_class,
            'pick_approach_x':            1.0,
            'pick_approach_y':            0.0,
        }]
    )

    return LaunchDescription([
        declare_slam,
        declare_map,
        declare_use_sim_time,
        declare_auto_nav,
        declare_target,
        slam_launch,
        nav2_launch,
        navigation_node,
    ])
