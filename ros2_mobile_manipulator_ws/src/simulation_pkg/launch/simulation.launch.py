"""simulation.launch.py — Phase 2: Robot Description + Simulation Launch"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description() -> LaunchDescription:

    pkg_share    = get_package_share_directory("simulation_pkg")
    gazebo_share = get_package_share_directory("gazebo_ros")

    urdf_file   = os.path.join(pkg_share, "urdf",   "mobile_manipulator.urdf.xacro")
    world_file  = os.path.join(pkg_share, "worlds", "pick_and_place.world")
    rviz_config = os.path.join(pkg_share, "rviz",   "default.rviz")

    # Pre-process xacro → URDF string at launch time (captures only stdout,
    # discards the harmless stderr "redefining pi" warning completely)
    xacro_result = subprocess.run(
        ["xacro", urdf_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,   # silence the pi-redefinition warning
        check=True,
    )
    robot_urdf = xacro_result.stdout.decode("utf-8")
    robot_description = {"robot_description": robot_urdf}

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    declare_gui          = DeclareLaunchArgument("gui",          default_value="true")
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_rviz         = DeclareLaunchArgument("rviz",         default_value="true")

    gui          = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz  = LaunchConfiguration("rviz")

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_file, "pause": "false"}.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui),
    )

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
                    "-x", "0.0", "-y", "0.0", "-z", "0.06", "-Y", "0.0",
                ],
                output="screen",
            )
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
        output="screen",
    )
    

    return LaunchDescription([
        declare_gui, declare_use_sim_time, declare_rviz,
        robot_state_publisher_node,
        gazebo_server, gazebo_client,
        spawn_robot, rviz_node,
    ])