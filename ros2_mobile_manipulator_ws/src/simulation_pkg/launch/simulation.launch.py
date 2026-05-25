"""simulation.launch.py

Phase 2 — Robot Description + Simulation Launch
Launches: robot_state_publisher, Gazebo (with pick_and_place world),
          spawns the robot URDF, and opens RViz2.

Usage:
    ros2 launch simulation_pkg simulation.launch.py
    ros2 launch simulation_pkg simulation.launch.py gui:=false   # headless
"""

import os
from pathlib import Path

from ament_python_package import get_package_share_directory  # type: ignore

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ------------------------------------------------------------------
    # Package paths
    # ------------------------------------------------------------------
    pkg_share = FindPackageShare("simulation_pkg")

    urdf_file = PathJoinSubstitution(
        [pkg_share, "urdf", "mobile_manipulator.urdf.xacro"]
    )
    world_file = PathJoinSubstitution(
        [pkg_share, "worlds", "pick_and_place.world"]
    )
    rviz_config = PathJoinSubstitution(
        [pkg_share, "rviz", "default.rviz"]
    )

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

    gui            = LaunchConfiguration("gui")
    use_sim_time   = LaunchConfiguration("use_sim_time")
    launch_rviz    = LaunchConfiguration("rviz")

    # ------------------------------------------------------------------
    # Robot description (xacro → URDF string)
    # ------------------------------------------------------------------
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_file,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------

    # 1. robot_state_publisher — broadcasts TF from URDF joint states
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
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gzserver.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "world": world_file,
            "pause": "false",
        }.items(),
    )

    # 3. Gazebo client (GUI) — conditional on gui:=true
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gzclient.launch.py",
                    ]
                )
            ]
        ),
        condition=IfCondition(gui),
    )

    # 4. Spawn the robot into Gazebo (delayed 2 s to let Gazebo start)
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

    # 5. RViz2 — conditional on rviz:=true
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
    # Assemble LaunchDescription
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
