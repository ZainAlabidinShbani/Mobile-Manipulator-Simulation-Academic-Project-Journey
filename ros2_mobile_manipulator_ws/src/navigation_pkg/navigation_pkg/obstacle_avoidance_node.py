import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.declare_parameter('safety_distance', 0.45)
        self.scan = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self._cmd_cb, 10)

    def _scan_cb(self, msg):
        self.scan = msg

    def _cmd_cb(self, msg):
        safe = Twist()
        safe.linear.x = msg.linear.x
        safe.angular.z = msg.angular.z
        if self.scan and self.scan.ranges:
            front = self._front_clearance(self.scan)
            if front < float(self.get_parameter('safety_distance').value):
                safe.linear.x = 0.0
                safe.angular.z = max(-1.0, min(1.0, msg.angular.z if abs(msg.angular.z) > 0.1 else 0.7))
        self.cmd_pub.publish(safe)

    def _front_clearance(self, scan):
        if not scan.ranges:
            return 10.0
        center = len(scan.ranges) // 2
        window = scan.ranges[max(0, center - 10):min(len(scan.ranges), center + 10)]
        values = [r for r in window if scan.range_min <= r <= scan.range_max]
        return min(values) if values else scan.range_max


def main():
    rclpy.init()
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
