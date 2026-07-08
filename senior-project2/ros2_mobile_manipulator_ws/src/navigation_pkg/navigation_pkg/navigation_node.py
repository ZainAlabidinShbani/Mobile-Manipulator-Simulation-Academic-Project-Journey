#!/usr/bin/env python3
"""
navigation_node.py

Autonomous navigation node for the mobile manipulator.
Listens to YOLO detections → drives to pick approach pose → notifies task manager.

Goals:
  pick_approach : (1.0, 0.0, 0.0)   — 1 m in front of pick table
  drop_approach : (-2.0, -2.5, 0.0) — in front of drop station
  home          : (0.0, 0.0, 0.0)   — start zone

Fixes applied:
  - PoseStamped header stamp now set (required by Nav2 for goal acceptance)
  - Shutdown traceback fixed: destroy_node() before rclpy.shutdown()
  - Added manual command topic test helper log on startup
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mobile_manipulator_msgs.msg import DetectionArray

import math


class NavigationNode(Node):

    # Pre-defined named goals in map frame
    GOALS = {
        'pick_approach': (1.0,   0.0,  0.0),
        'drop_approach': (-2.0, -2.5,  0.0),
        'home':          (0.0,   0.0,  0.0),
    }

    def __init__(self):
        super().__init__('navigation_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('auto_navigate_on_detection', True)
        self.declare_parameter('target_class', 'cube')
        self.declare_parameter('pick_approach_x', 1.0)
        self.declare_parameter('pick_approach_y', 0.0)

        self.auto_nav     = self.get_parameter('auto_navigate_on_detection').value
        self.target_class = self.get_parameter('target_class').value

        # ── State ───────────────────────────────────────────────────────────
        self._navigating         = False
        self._current_goal       = None
        self._detection_received = False

        # ── Nav2 Action Client ───────────────────────────────────────────────
        self._cb_group = ReentrantCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self.create_subscription(
            DetectionArray, '/detections',
            self._detection_callback, 10
        )
        self.create_subscription(
            String, '/navigation/command',
            self._command_callback, 10,
            callback_group=self._cb_group
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, '/navigation/status', 10)

        self.get_logger().info('NavigationNode started. Waiting for Nav2...')
        self._nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server ready ✓')
        self.get_logger().info(
            'To test movement, publish a goal name to /navigation/command:\n'
            '  ros2 topic pub --once /navigation/command std_msgs/String "{data: home}"'
        )
        self._publish_status('READY')

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def _detection_callback(self, msg: DetectionArray):
        """Trigger navigation when target object is detected."""
        if not self.auto_nav or self._navigating or self._detection_received:
            return

        for det in msg.detections:
            if det.class_id == self.target_class and det.confidence > 0.7:
                self.get_logger().info(
                    f'Target "{self.target_class}" detected (conf={det.confidence:.2f}) '
                    f'→ navigating to pick_approach'
                )
                self._detection_received = True
                self.navigate_to('pick_approach')
                break

    def _command_callback(self, msg: String):
        """Accept manual navigation commands: pick_approach | drop_approach | home."""
        goal_name = msg.data.strip()
        if goal_name in self.GOALS:
            self.get_logger().info(f'Command received: navigate to "{goal_name}"')
            self.navigate_to(goal_name)
        else:
            self.get_logger().warn(
                f'Unknown goal name: "{goal_name}". '
                f'Valid options: {list(self.GOALS.keys())}'
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Navigation helpers
    # ─────────────────────────────────────────────────────────────────────────

    def navigate_to(self, goal_name: str):
        """Send a NavigateToPose goal by name."""
        x, y, yaw = self.GOALS[goal_name]
        self._send_goal(x, y, yaw, goal_name)

    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0):
        """Send a NavigateToPose goal by coordinates."""
        self._send_goal(x, y, yaw, label=f'({x:.2f},{y:.2f})')

    def _send_goal(self, x: float, y: float, yaw: float, label: str = ''):
        if self._navigating:
            self.get_logger().warn('Already navigating — ignoring new goal.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(x, y, yaw)
        # Stamp the header with current ROS time (required by Nav2)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._navigating   = True
        self._current_goal = label
        self._publish_status(f'NAVIGATING_TO_{label.upper()}')
        self.get_logger().info(f'Sending goal → {label}  x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2!')
            self._navigating = False
            self._publish_status('GOAL_REJECTED')
            return
        self.get_logger().info('Goal accepted ✓')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        status = result.status
        self._navigating = False

        if status == 4:   # SUCCEEDED
            self.get_logger().info(f'Reached goal: {self._current_goal} ✓')
            self._publish_status(f'REACHED_{self._current_goal.upper()}')
        else:
            self.get_logger().warn(f'Navigation failed — status={status}')
            self._detection_received = False  # allow retry
            self._publish_status(f'FAILED_status{status}')

    def _feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        if dist > 0.1:
            self.get_logger().info(
                f'Distance remaining: {dist:.2f} m',
                throttle_duration_sec=2.0
            )

    # ─────────────────────────────────────────────────────────────────────────
    # Utilities
    # ─────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _make_pose(x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # Note: stamp is set in _send_goal using live clock
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        # Convert yaw to quaternion (rotation around Z axis)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown — avoids traceback on Ctrl+C
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
