"""
simulation.launch.py
====================
Gazebo full-simulation launch for the Mobile Manipulator.

Chain:
  gzserver  +  gzclient
  robot_state_publisher   (robot_description from xacro)
  joint_state_publisher   (publishes /joint_states for arm joints
                           until ros2_control takes over)
  spawn_entity            (spawns robot into Gazebo)
  rviz2                   (optional)

Note on /odom:
  - In Gazebo mode the diff_drive plugin publishes /odom and the
    odom->base_footprint TF automatically once the robot is spawned.
  - The static_tf fallback is NOT used here so it doesn't conflict
    with the real odometry coming from the plugin.
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
    rviz_config = os.path.join(pkg_sim, 'rviz',   'default.rviz')

    # ── Launch arguments ──────────────────────────────────────────────
    declare_gui          = DeclareLaunchArgument('gui',          default_value='true')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_rviz         = DeclareLaunchArgument('rviz',         default_value='true')

    gui          = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz  = LaunchConfiguration('rviz')

    # ── robot_description: process xacro at launch time ───────────────
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── Nodes ──────────────────────────────────────────────────────────

    # 1. robot_state_publisher — publishes TF tree from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # 2. joint_state_publisher — publishes /joint_states so RViz can
    #    display the arm links before ros2_control takes over.
    #    (use joint_state_publisher, NOT the _gui version, in sim mode)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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

    # 5. Spawn robot — wait 5 s for Gazebo physics to be ready
    spawn_robot = TimerAction(
        period=5.0,
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

    # 6. Load ros2_control controllers after spawn (10 s total margin)
    load_joint_state_broadcaster = TimerAction(
        period=12.0,
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
        period=14.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller',
                     '--set-state', 'active',
                     'arm_position_controller'],
                output='screen',
            )
        ],
    )

    # 7. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_gui,
        declare_use_sim_time,
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
