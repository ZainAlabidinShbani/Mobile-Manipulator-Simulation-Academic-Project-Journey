"""
simulation.launch.py
====================
Gazebo full-simulation launch for the Mobile Manipulator.

Chain:
  gzserver  +  gzclient  (gzclient skipped when gui:=false)
  robot_state_publisher   (use_sim_time driven by launch arg)
  joint_state_publisher
  spawn_entity            (spawns robot into Gazebo after 12 s delay)
  rviz2                   (skipped when rviz:=false)
  ros2_control loaders    (20 s / 23 s delays after start)

Rendering fix (VM / WSL2 / software GPU):
  LIBGL_ALWAYS_SOFTWARE=1  forces Mesa software rasteriser.
  OGRE_RTT_MODE=Copy       fixes RViz Ogre render-texture crash.
  software_render:=false   disables these overrides on real GPU machines.

Usage examples:
  # Normal (software rendering ON by default):
  ros2 launch simulation_pkg simulation.launch.py

  # Real GPU machine:
  ros2 launch simulation_pkg simulation.launch.py software_render:=false

  # Headless (no Gazebo GUI, no RViz) — useful for CI or SSH sessions:
  ros2 launch simulation_pkg simulation.launch.py gui:=false rviz:=false

  # Headless + real GPU:
  ros2 launch simulation_pkg simulation.launch.py gui:=false rviz:=false software_render:=false
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
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_sim      = get_package_share_directory('simulation_pkg')
    pkg_gazebo   = get_package_share_directory('gazebo_ros')
    pkg_assembly = get_package_share_directory('assembly_of_robot')

    xacro_file  = os.path.join(pkg_sim,  'urdf',   'mobile_manipulator.urdf.xacro')
    world_file  = os.path.join(pkg_sim,  'worlds', 'pick_and_place.world')
    rviz_config = os.path.join(pkg_sim,  'rviz',   'assembly_of_robot.rviz')
    controllers_file = os.path.join(pkg_sim, 'config', 'controllers.yaml')

    # ── Declare launch arguments ──────────────────────────────────────
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Launch Gazebo GUI client (gzclient). Set false for headless/SSH.')

    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2. Set false for headless/CI.')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock for all nodes.')

    declare_software_render = DeclareLaunchArgument(
        'software_render', default_value='true',
        description=(
            'Force Mesa software OpenGL renderer '
            '(LIBGL_ALWAYS_SOFTWARE=1 + OGRE_RTT_MODE=Copy). '
            'Set false on machines with a real dedicated GPU.'))

    gui              = LaunchConfiguration('gui')
    launch_rviz      = LaunchConfiguration('rviz')
    use_sim_time     = LaunchConfiguration('use_sim_time')
    software_render  = LaunchConfiguration('software_render')

    # ── Environment variables ─────────────────────────────────────────

    # Always set: GAZEBO_MODEL_PATH so Gazebo finds assembly_of_robot meshes
    existing_model_path    = os.environ.get('GAZEBO_MODEL_PATH', '')
    existing_resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')

    new_model_path = (
        pkg_assembly + ':' + existing_model_path
        if existing_model_path else pkg_assembly
    )
    new_resource_path = (
        pkg_assembly + ':' + existing_resource_path
        if existing_resource_path else pkg_assembly
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', new_model_path)

    set_gazebo_resource_path = SetEnvironmentVariable(
        'GAZEBO_RESOURCE_PATH', new_resource_path)

    # DISPLAY fallback — prevents Qt "wayland plugin not found" error
    # Only set if DISPLAY is not already set in the environment
    set_display = SetEnvironmentVariable(
        'DISPLAY',
        value=os.environ.get('DISPLAY', ':0'),
    )

    # Software rendering env vars (active when software_render:=true)
    set_libgl_software = SetEnvironmentVariable(
        'LIBGL_ALWAYS_SOFTWARE', '1',
        condition=IfCondition(software_render),
    )

    set_ogre_rtt = SetEnvironmentVariable(
        'OGRE_RTT_MODE', 'Copy',
        condition=IfCondition(software_render),
    )

    # Mesa GL version hint — helps software renderer claim OpenGL 3.3
    set_mesa_gl = SetEnvironmentVariable(
        'MESA_GL_VERSION_OVERRIDE', '3.3',
        condition=IfCondition(software_render),
    )

    # ── robot_description ────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_yaml:=', controllers_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── 1. robot_state_publisher ─────────────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ── 2. joint_state_publisher ──────────────────────────────────────
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 3. Gazebo server ──────────────────────────────────────────────
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'pause': 'false',
            'verbose': 'true',          # prints Gazebo errors to console
        }.items(),
    )

    # ── 4. Gazebo client (GUI) ────────────────────────────────────────
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui),
    )

    # ── 5. Spawn robot (12 s delay — gives gzserver time to fully init) ─
    spawn_robot = TimerAction(
        period=12.0,
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

    # ── 6. Load ros2_control controllers (after spawn settles) ────────
    load_joint_state_broadcaster = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    'joint_state_broadcaster',
                ],
                output='screen',
            )
        ],
    )

    load_arm_controller = TimerAction(
        period=23.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    'arm_position_controller',
                ],
                output='screen',
            )
        ],
    )

    # ── 7. RViz2 ──────────────────────────────────────────────────────
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
        # ── args must be declared before referenced in conditions ──
        declare_gui,
        declare_rviz,
        declare_use_sim_time,
        declare_software_render,
        # ── env vars ──
        set_gazebo_model_path,
        set_gazebo_resource_path,
        set_display,
        set_libgl_software,        # conditional: only when software_render=true
        set_ogre_rtt,              # conditional: only when software_render=true
        set_mesa_gl,               # conditional: only when software_render=true
        # ── nodes ──
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller,
        rviz_node,
    ])
