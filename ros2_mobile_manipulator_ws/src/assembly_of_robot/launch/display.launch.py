from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("assembly_of_robot")
    urdf_path = os.path.join(pkg_share, "urdf", "assembly_of_robot.urdf")
    rviz_config = os.path.join(pkg_share, "rviz", "display.rviz")

    with open(urdf_path, "r") as urdf_file:
        robot_description = urdf_file.read()

    return LaunchDescription(
        [
            # Publish a static identity transform odom -> base_link
            # so RViz works without a running navigation stack
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_odom_to_base",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
            ),
            # Publish joint states for all movable joints (wheels + arm)
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
            ),
            # Publish TF tree from URDF
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            # Launch RViz2 with preconfigured display settings
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
            ),
        ]
    )
