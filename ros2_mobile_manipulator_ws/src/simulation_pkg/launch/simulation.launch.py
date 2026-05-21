import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        os.path.join(pkg_share, 'urdf', 'mobile_manipulator.urdf.xacro'),
    ])

    declare_world = DeclareLaunchArgument('world', default_value=os.path.join(pkg_share, 'worlds', 'pick_and_place.world'))
    declare_rviz = DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_share, 'rviz', 'mobile_manipulator.rviz'))
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-param', 'robot_description', '-entity', 'mobile_manipulator'],
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )
    gazebo_bridge = Node(package='simulation_pkg', executable='gazebo_bridge_node', parameters=[os.path.join(pkg_share, 'config', 'simulation.yaml')], output='screen')
    rviz_bridge = Node(package='simulation_pkg', executable='rviz_bridge_node', parameters=[os.path.join(pkg_share, 'config', 'simulation.yaml')], output='screen')
    return LaunchDescription([declare_world, declare_rviz, declare_use_rviz, gazebo, state_publisher, spawn_entity, gazebo_bridge, rviz_bridge, rviz])
