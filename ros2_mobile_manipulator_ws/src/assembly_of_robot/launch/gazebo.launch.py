from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("assembly_of_robot")
    urdf_path = os.path.join(pkg_share, "urdf", "assembly_of_robot.urdf")

    gazebo_launch_path = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch_path)),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_footprint_base",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_entity",
                arguments=["-file", urdf_path, "-entity", "assembly_of_robot"],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "topic", "pub", "-1", "/calibrated", "std_msgs/msg/Bool", "true"],
                output="screen",
            ),
        ]
    )
