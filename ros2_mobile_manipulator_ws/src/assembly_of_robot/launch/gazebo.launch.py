"""
gazebo.launch.py — Gazebo simulation for assembly_of_robot
==========================================================
Launches:
  1. Gazebo (empty world) via ros_gz_sim (ROS 2 Humble standard)
  2. robot_state_publisher  — broadcasts URDF TF tree
  3. joint_state_publisher  — publishes zero joint states
  4. spawn_entity  — spawns the robot URDF into the running Gazebo world

Usage:
  ros2 launch assembly_of_robot gazebo.launch.py

Notes:
  - The diff_drive Gazebo plugin (in the URDF) will automatically
    subscribe to /cmd_vel and publish /odom once the robot is spawned.
  - To drive the robot: ros2 topic pub /cmd_vel geometry_msgs/Twist ...
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('assembly_of_robot')
    urdf_path = os.path.join(pkg_share, 'urdf', 'assembly_of_robot.urdf')

    # Allow overriding the URDF path from command line
    declare_urdf_arg = DeclareLaunchArgument(
        name='urdf',
        default_value=urdf_path,
        description='Absolute path to the robot URDF file'
    )

    robot_description = ParameterValue(
        Command(['cat ', LaunchConfiguration('urdf')]),
        value_type=str
    )

    # 1. Launch Gazebo empty world (ROS 2 Humble: ros_gz_sim package)
    gazebo_launch_path = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 2. robot_state_publisher — publishes TF from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 3. joint_state_publisher — publishes zero positions for fixed/passive joints
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Spawn robot into Gazebo from the robot_description topic
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-name', 'assembly_of_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'   # spawn slightly above ground to avoid collision at t=0
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_urdf_arg,
        gazebo,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_robot_node,
    ])
