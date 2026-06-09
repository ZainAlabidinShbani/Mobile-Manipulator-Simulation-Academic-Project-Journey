"""
simulation.launch.py
====================
Gazebo full-simulation launch for the Mobile Manipulator.

Chain:
  gzserver  +  gzclient
  robot_state_publisher   (use_sim_time=false so robot_description is
                           available immediately for spawn_entity)
  joint_state_publisher
  spawn_entity            (spawns robot into Gazebo, delayed 8 s)
  rviz2                   (use_sim_time=false, same config as rviz_display)

Notes:
  - robot_state_publisher and rviz2 use use_sim_time=false.
    This matches rviz_display.launch.py behaviour (no clock dependency)
    and ensures robot_description is published before spawn_entity runs.
  - The diff_drive plugin publishes /odom and odom->base_footprint TF
    automatically once the robot is spawned.
  - No static_tf fallback here (would conflict with diff_drive odom TF).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_sim    = get_package_share_directory('simulation_pkg')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    xacro_file  = os.path.join(pkg_sim, 'urdf',   'mobile_manipulator.urdf.xacro')
    world_file  = os.path.join(pkg_sim, 'worlds', 'pick_and_place.world')
    rviz_config = os.path.join(pkg_sim, 'rviz',   'assembly_of_robot.rviz')

    # ── Launch arguments ──────────────────────────────────────────────
    declare_gui  = DeclareLaunchArgument('gui',  default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')

    gui         = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('rviz')

    # ── robot_description ─────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── Nodes ──────────────────────────────────────────────────────────

    # 1. robot_state_publisher
    #    use_sim_time=FALSE → publishes robot_description immediately,
    #    no dependency on Gazebo /clock topic.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
    )

    # 2. joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # 3. Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'pause': 'false'}.items(),
    )

    # 4. Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    # 5. Spawn robot — wait 8 s for Gazebo physics + ROS bridge to be ready
    spawn_robot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_robot',
                arguments=[
                    '-topic', '/robot_description',
                    '-entity', 'mobile_manipulator',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.05',
                    '-Y', '0.0',
                ],
                output='screen',
            )
        ],
    )

    # 6. Load ros2_control controllers
    load_joint_state_broadcaster = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active',
                     'joint_state_broadcaster'],
                output='screen',
            )
        ],
    )

    load_arm_controller = TimerAction(
        period=17.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active',
                     'arm_position_controller'],
                output='screen',
            )
        ],
    )

    # 7. RViz2 — use_sim_time=FALSE (matches rviz_display behaviour)
    #    Same assembly_of_robot.rviz config used in rviz_display.launch.py
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(launch_rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
        rviz_node,
    ])
