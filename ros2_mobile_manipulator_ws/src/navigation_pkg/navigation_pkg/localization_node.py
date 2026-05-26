import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.robot_pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def _odom_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = msg.header.frame_id
        pose.pose = msg.pose.pose
        self.robot_pose_pub.publish(pose)


def main():
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
