import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class GazeboBridgeNode(Node):
    def __init__(self):
        super().__init__('gazebo_bridge_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('scan_frame', 'base_laser')
        self.declare_parameter('world_size', 6.0)
        self.declare_parameter('obstacle_x', 1.4)
        self.declare_parameter('obstacle_y', 0.0)
        self.declare_parameter('obstacle_radius', 0.35)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_twist = Twist()
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self._tick)

    def _cmd_cb(self, msg: Twist):
        self.last_twist = msg

    def _ray_to_circle(self, angle: float, cx: float, cy: float, radius: float, max_range: float) -> float:
        dx = math.cos(angle)
        dy = math.sin(angle)
        ox = self.x
        oy = self.y
        fx = ox - cx
        fy = oy - cy
        a = dx * dx + dy * dy
        b = 2.0 * (fx * dx + fy * dy)
        c = fx * fx + fy * fy - radius * radius
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return max_range
        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) / (2.0 * a)
        t2 = (-b + sqrt_disc) / (2.0 * a)
        candidates = [t for t in (t1, t2) if 0.0 <= t <= max_range]
        return min(candidates) if candidates else max_range

    def _tick(self):
        dt = 0.05
        linear = max(-0.6, min(0.6, self.last_twist.linear.x))
        angular = max(-1.5, min(1.5, self.last_twist.angular.z))
        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + angular * dt), math.cos(self.yaw + angular * dt))

        stamp = self.get_clock().now().to_msg()
        base_frame = self.get_parameter('base_frame').value
        odom_frame = self.get_parameter('odom_frame').value
        scan_frame = self.get_parameter('scan_frame').value

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist = self.last_twist
        self.odom_pub.publish(odom)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = odom_frame
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(pose)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = odom_frame
        tf.child_frame_id = base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = scan_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.range_min = 0.05
        scan.range_max = 5.0
        obstacle_x = float(self.get_parameter('obstacle_x').value)
        obstacle_y = float(self.get_parameter('obstacle_y').value)
        obstacle_r = float(self.get_parameter('obstacle_radius').value)
        scan.ranges = [self._ray_to_circle(scan.angle_min + i * scan.angle_increment, obstacle_x, obstacle_y, obstacle_r, scan.range_max) for i in range(int((scan.angle_max - scan.angle_min) / scan.angle_increment))]
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = GazeboBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
