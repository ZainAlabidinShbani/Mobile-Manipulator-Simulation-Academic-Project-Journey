import os
from pathlib import Path
from tempfile import gettempdir, mkstemp

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnShutdown
from launch.logging import get_logger
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


LOGGER = get_logger('simulation_launch')


def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_pkg')
    gazebo_share = get_package_share_directory('gazebo_ros')
    world = LaunchConfiguration('world')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_world = DeclareLaunchArgument('world', default_value=os.path.join(pkg_share, 'worlds', 'pick_and_place.world'))
    declare_rviz = DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_share, 'rviz', 'mobile_manipulator.rviz'))
    declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items(),
    )

    def launch_setup(context, *args, **kwargs):
        robot_description = Command([
            FindExecutable(name='xacro'),
            ' ',
            os.path.join(pkg_share, 'urdf', 'mobile_manipulator.urdf.xacro'),
        ]).perform(context)

        urdf_fd, robot_urdf_path = mkstemp(suffix='.urdf', dir=gettempdir(), text=True)
        with os.fdopen(urdf_fd, 'w') as urdf_file:
            urdf_file.write(robot_description)

        def cleanup_robot_urdf(context, *args, **kwargs):
            try:
                Path(robot_urdf_path).unlink()
            except FileNotFoundError:
                LOGGER.warning(f'Temporary URDF file not found during cleanup: {robot_urdf_path}')
            except OSError as exc:
                LOGGER.warning(f'Failed to remove temporary URDF file {robot_urdf_path}: {exc}')
            return []

        state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
            output='screen',
        )
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', robot_urdf_path, '-entity', 'mobile_manipulator'],
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
        cleanup_handler = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_robot_urdf)]))
        return [gazebo, state_publisher, spawn_entity, gazebo_bridge, rviz_bridge, rviz, cleanup_handler]

    return LaunchDescription([declare_world, declare_rviz, declare_use_rviz, OpaqueFunction(function=launch_setup)])
