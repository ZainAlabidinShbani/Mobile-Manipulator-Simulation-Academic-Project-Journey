import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_pkg')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')

    # ------------------------------------------------------------------ #
    # Resolve Gazebo resource/model paths so RTShaderSystem can find libs  #
    # ------------------------------------------------------------------ #
    gazebo_resource_path = '/usr/share/gazebo-11'
    gazebo_model_path = os.path.join(gazebo_resource_path, 'models')

    # Extend existing env vars rather than overwrite them
    existing_resource = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    existing_model = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_resource = (gazebo_resource_path + ':' + existing_resource).rstrip(':')
    new_model = (gazebo_model_path + ':' + existing_model).rstrip(':')

    set_gazebo_resource = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', new_resource)
    set_gazebo_model = SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model)

    # Force X11 display backend so RViz2 does not try (and fail) Wayland  #
    set_display = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    set_mesa = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '0')

    # ------------------------------------------------------------------ #
    # Robot description                                                    #
    # ------------------------------------------------------------------ #
    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        os.path.join(pkg_share, 'urdf', 'mobile_manipulator.urdf.xacro'),
    ])

    # ------------------------------------------------------------------ #
    # Launch arguments                                                     #
    # ------------------------------------------------------------------ #
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'pick_and_place.world'),
        description='Full path to Gazebo world file',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'assembly_of_robot.rviz'),
        description='Full path to RViz2 config file',
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2',
    )

    # ------------------------------------------------------------------ #
    # Gazebo (server + client via gazebo_ros launch)                       #
    # ------------------------------------------------------------------ #
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
        }.items(),
    )

    # ------------------------------------------------------------------ #
    # ROS 2 nodes                                                          #
    # ------------------------------------------------------------------ #
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-param', 'robot_description', '-entity', 'mobile_manipulator'],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        output='screen',
    )

    gazebo_bridge = Node(
        package='simulation_pkg',
        executable='gazebo_bridge_node',
        parameters=[os.path.join(pkg_share, 'config', 'simulation.yaml')],
        output='screen',
    )

    rviz_bridge = Node(
        package='simulation_pkg',
        executable='rviz_bridge_node',
        parameters=[os.path.join(pkg_share, 'config', 'simulation.yaml')],
        output='screen',
    )

    return LaunchDescription([
        # Environment must be set before any process is spawned
        set_gazebo_resource,
        set_gazebo_model,
        set_display,
        set_mesa,
        # Declare args
        declare_world,
        declare_rviz,
        declare_use_rviz,
        # Launch everything
        gazebo,
        state_publisher,
        joint_state_publisher,
        spawn_entity,
        gazebo_bridge,
        rviz_bridge,
        rviz,
    ])
