"""simulation.launch.py

Gazebo full-simulation launch for the Mobile Manipulator.

Robot description comes from simulation_pkg/urdf/mobile_manipulator.urdf.xacro
which is now geometrically identical to assembly_of_robot/urdf/assembly_of_robot.urdf.

Chain:
  gzserver  +  gzclient  -->  robot_state_publisher  -->  spawn_entity
  -->  joint_state_broadcaster  -->  arm_position_controller  -->  rviz2
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    pkg_share    = get_package_share_directory("simulation_pkg")
    gazebo_share = get_package_share_directory("gazebo_ros")

    urdf_file   = os.path.join(pkg_share, "urdf",   "mobile_manipulator.urdf.xacro")
    world_file  = os.path.join(pkg_share, "worlds", "pick_and_place.world")
    rviz_config = os.path.join(pkg_share, "rviz",   "default.rviz")

    # Process xacro -> plain URDF string at launch time
    xacro_result = subprocess.run(
        ["xacro", urdf_file],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
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
    # Core nodes
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

    # Spawn robot – wait 5 s for Gazebo physics to be ready
    spawn_robot = TimerAction(
        period=5.0,
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
                    "-z", "0.05",   # lift slightly above ground (wheel radius)
                    "-Y", "0.0",
                ],
                output="screen",
            )
        ],
    )

    # ------------------------------------------------------------------
    # ros2_control – load controllers after spawn
    # ------------------------------------------------------------------
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_arm_position_controller = ExecuteProcess(
        cmd=[
            "ros2", "control", "load_controller",
            "--set-state", "active",
            "arm_position_controller",
        ],
        output="screen",
    )

    # Chain: spawn done -> load joint_state_broadcaster -> load arm controller
    # We trigger on spawn_robot's internal Node exit via TimerAction workaround:
    # simplest reliable approach is a second timer after spawn.
    load_controllers = TimerAction(
        period=10.0,   # 5 s Gazebo + 5 s spawn = safe margin
        actions=[
            load_joint_state_broadcaster,
            load_arm_position_controller,
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
        declare_gui,
        declare_use_sim_time,
        declare_rviz,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        load_controllers,
        rviz_node,
    ])
