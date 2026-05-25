"""simulation.launch.py

Phase 2 — Robot Description + Simulation Launch
Launches: robot_state_publisher, Gazebo (with pick_and_place world),
          spawns the robot URDF, and opens RViz2.

Usage:
    ros2 launch simulation_pkg simulation.launch.py
    ros2 launch simulation_pkg simulation.launch.py gui:=false   # headless
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ------------------------------------------------------------------
    # Package share directory (resolved eagerly at import time via
    # ament_index — avoids substitution concatenation bugs)
    # ------------------------------------------------------------------
    pkg_share  = get_package_share_directory("simulation_pkg")
    gazebo_share = get_package_share_directory("gazebo_ros")

    urdf_file   = os.path.join(pkg_share, "urdf",   "mobile_manipulator.urdf.xacro")
    world_file  = os.path.join(pkg_share, "worlds",  "pick_and_place.world")
    rviz_config = os.path.join(pkg_share, "rviz",   "default.rviz")

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo with GUI (true) or headless (false)",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock from Gazebo",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 visualisation",
    )

    gui          = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz  = LaunchConfiguration("rviz")

    # ------------------------------------------------------------------
    # Robot description — xacro → URDF string
    # Command list is concatenated with NO automatic separator.
    # The space " " between executable and file path is mandatory.
    # ------------------------------------------------------------------
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",          # <-- explicit space between 'xacro' and the path
            urdf_file,    # plain string from os.path.join (no substitution)
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------

    # 1. robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 2. Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "pause": "false",
        }.items(),
    )

    # 3. Gazebo client (GUI) — only when gui:=true
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui),
    )

    # 4. Spawn robot (2 s delay to let gzserver initialise)
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_robot",
                arguments=[
                    "-topic", "/robot_description",
                    "-entity", "mobile_manipulator",
                    "-x", "0.0",
                    "-y", "0.0",
                    "-z", "0.06",
                    "-Y", "0.0",
                ],
                output="screen",
            )
        ],
    )

    # 5. RViz2 — only when rviz:=true
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
        output="screen",
    )

    # ------------------------------------------------------------------
    # Assemble
    # ------------------------------------------------------------------
    return LaunchDescription(
        [
            declare_gui,
            declare_use_sim_time,
            declare_rviz,
            robot_state_publisher_node,
            gazebo_server,
            gazebo_client,
            spawn_robot,
            rviz_node,
        ]
    )
