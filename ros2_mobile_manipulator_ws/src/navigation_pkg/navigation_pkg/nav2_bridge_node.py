import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node


class Nav2BridgeNode(Node):
    def __init__(self):
        super().__init__('nav2_bridge_node')
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('linear_gain', 0.6)
        self.declare_parameter('angular_gain', 1.4)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('use_nav2', False)
        self.current_pose = None
        self.active_goal = None
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.create_subscription(PoseStamped, '/navigate_to_pose', self._goal_cb, 10)
        self.create_subscription(PoseStamped, '/robot_pose', self._pose_cb, 10)
        self.timer = self.create_timer(0.1, self._tick)

    def _goal_cb(self, msg):
        self.active_goal = msg
        self.get_logger().info(f'Navigation goal received for frame {msg.header.frame_id}')

    def _pose_cb(self, msg):
        self.current_pose = msg

    def _tick(self):
        twist = Twist()
        if self.current_pose is None or self.active_goal is None:
            self.cmd_pub.publish(twist)
            return
        dx = self.active_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.active_goal.pose.position.y - self.current_pose.pose.position.y
        distance = math.hypot(dx, dy)
        if distance <= float(self.get_parameter('goal_tolerance').value):
            self.active_goal = None
            self.cmd_pub.publish(twist)
            return
        target_heading = math.atan2(dy, dx)
        current_yaw = self._yaw_from_quaternion(self.current_pose.pose.orientation)
        heading_error = math.atan2(math.sin(target_heading - current_yaw), math.cos(target_heading - current_yaw))
        linear = min(float(self.get_parameter('max_linear_speed').value), float(self.get_parameter('linear_gain').value) * distance)
        linear *= max(0.0, math.cos(heading_error))
        angular = max(-float(self.get_parameter('max_angular_speed').value), min(float(self.get_parameter('max_angular_speed').value), float(self.get_parameter('angular_gain').value) * heading_error))
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    @staticmethod
    def _yaw_from_quaternion(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Nav2BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
