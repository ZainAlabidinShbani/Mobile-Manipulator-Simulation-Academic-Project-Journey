"""RViz-only display launch for the SolidWorks URDF."""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("simulation_pkg")
    urdf_file = os.path.join(pkg_share, "urdf", "mobile_manipulator.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "assembly_of_robot.rviz")

    xacro_result = subprocess.run(
        ["xacro", urdf_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=True,
    )
    robot_urdf = xacro_result.stdout.decode("utf-8")

    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="false")
    declare_rviz_config = DeclareLaunchArgument("rviz_config", default_value=rviz_config)

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config_arg = LaunchConfiguration("rviz_config")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_urdf}, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_arg],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz_config,
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
        ]
    )
