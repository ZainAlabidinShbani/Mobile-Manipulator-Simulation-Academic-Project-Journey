"""RViz-only display launch for the Mobile Manipulator URDF.

Fixes:
  - Reads the URDF from assembly_of_robot package (not a missing xacro in simulation_pkg).
  - Uses open() instead of subprocess/xacro because the file is a plain .urdf.
  - Sets Fixed Frame to base_link so RViz works without a nav stack publishing odom.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # --- Paths -----------------------------------------------------------
    # URDF lives in assembly_of_robot, not simulation_pkg
    robot_pkg_share = get_package_share_directory("assembly_of_robot")
    sim_pkg_share   = get_package_share_directory("simulation_pkg")

    urdf_file   = os.path.join(robot_pkg_share, "urdf", "assembly_of_robot.urdf")
    rviz_config = os.path.join(sim_pkg_share,   "rviz", "default.rviz")

    # Read the plain URDF file (no xacro processing needed)
    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # --- Launch arguments ------------------------------------------------
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=rviz_config,
        description="Full path to the RViz config file",
    )

    use_sim_time    = LaunchConfiguration("use_sim_time")
    rviz_config_arg = LaunchConfiguration("rviz_config")

    # --- Nodes -----------------------------------------------------------
    # 1. Publishes /tf and /tf_static from the URDF joint tree
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # 2. GUI slider panel to move the revolute/continuous joints interactively
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # 3. RViz2 visualiser
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
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
