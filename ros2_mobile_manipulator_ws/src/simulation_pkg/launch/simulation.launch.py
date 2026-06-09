"""
simulation.launch.py
====================
Gazebo full-simulation launch for the Mobile Manipulator.

Chain:
  gzserver  +  gzclient
  robot_state_publisher   (use_sim_time=false)
  joint_state_publisher
  spawn_entity            (spawns robot into Gazebo after 8 s delay)
  rviz2                   (use_sim_time=false, full assembly_of_robot.rviz config)

Fix for Gazebo spawn:
  The xacro references meshes via  package://assembly_of_robot.
  Gazebo must be able to resolve that package at spawn time.
  We set GAZEBO_MODEL_PATH to include the assembly_of_robot share dir
  so that libgazebo_ros_pkgs resolves package:// URIs correctly.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_sim         = get_package_share_directory('simulation_pkg')
    pkg_gazebo      = get_package_share_directory('gazebo_ros')
    pkg_assembly    = get_package_share_directory('assembly_of_robot')

    xacro_file  = os.path.join(pkg_sim,     'urdf',   'mobile_manipulator.urdf.xacro')
    world_file  = os.path.join(pkg_sim,     'worlds', 'pick_and_place.world')
    rviz_config = os.path.join(pkg_sim,     'rviz',   'assembly_of_robot.rviz')

    # ── Make Gazebo aware of assembly_of_robot meshes ──────────────────
    # The xacro uses  package://assembly_of_robot/meshes/...  URIs.
    # Gazebo resolves package:// via GAZEBO_MODEL_PATH / ROS_PACKAGE_PATH.
    # Prepending the share directory of assembly_of_robot guarantees that
    # the meshes are found when spawn_entity loads the SDF/URDF.
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = pkg_assembly + ':' + existing_model_path if existing_model_path else pkg_assembly

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=new_model_path,
    )

    # ── Launch arguments ──────────────────────────────────────────────
    declare_gui  = DeclareLaunchArgument('gui',  default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='true')

    gui         = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('rviz')

    # ── robot_description ────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── Nodes ────────────────────────────────────────────────────────

    # 1. robot_state_publisher  (use_sim_time=False)
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

    # 5. Spawn robot — wait 8 s for Gazebo to be fully ready
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

    # 6. Load ros2_control controllers (after spawn)
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

    # 7. RViz2  (use_sim_time=False — same as rviz_display.launch.py)
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
        set_gazebo_model_path,          # must be first
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
