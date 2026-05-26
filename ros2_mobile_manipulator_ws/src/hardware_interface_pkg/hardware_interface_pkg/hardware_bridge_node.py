import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HardwareBridgeNode(Node):
    def __init__(self):
        super().__init__('hardware_bridge_node')
        self.joint_pub = self.create_publisher(JointState, '/hardware/joint_commands', 10)
        self.cmd_pub = self.create_publisher(Twist, '/hardware/cmd_vel', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)

    def _joint_cb(self, msg):
        self.joint_pub.publish(msg)

    def _cmd_cb(self, msg):
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = HardwareBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
