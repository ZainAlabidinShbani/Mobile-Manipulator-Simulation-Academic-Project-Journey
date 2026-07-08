import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mobile_manipulator_msgs.srv import SolveIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        self.declare_parameter('joint_names', ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4'])
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        self.ik_client = self.create_client(SolveIK, 'solve_ik')
        self.traj_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)
        self.create_subscription(PoseStamped, '/grasp_target_pose', lambda msg: self._handle_pose(msg, 'grasp'), 10)
        self.create_subscription(PoseStamped, '/dropoff_target_pose', lambda msg: self._handle_pose(msg, 'place'), 10)

    def _joint_state_cb(self, msg):
        if msg.position:
            self.current_joints = list(msg.position[:4]) + [0.0] * max(0, 4 - len(msg.position))

    def _handle_pose(self, msg, mode):
        if not self.ik_client.service_is_ready():
            self.get_logger().warn('solve_ik service is not available yet.')
            return
        request = SolveIK.Request()
        request.target_pose = msg
        request.seed_joint_positions = self.current_joints
        future = self.ik_client.call_async(request)
        future.add_done_callback(lambda fut: self._on_ik_response(fut, mode))

    def _on_ik_response(self, future, mode):
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime logging only
            self.get_logger().error(f'IK request failed: {exc}')
            return
        if not response.success:
            self.get_logger().warn(f'IK failed for {mode} target: {response.message}')
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.get_parameter('joint_names').value)
        start = self.current_joints[:4]
        goal = list(response.joint_positions[:4])
        for i in range(1, 16):
            alpha = i / 15.0
            point = JointTrajectoryPoint()
            point.positions = [s + (g - s) * alpha for s, g in zip(start, goal)]
            point.time_from_start.sec = int(2.0 * alpha)
            point.time_from_start.nanosec = int((2.0 * alpha - point.time_from_start.sec) * 1e9)
            trajectory.points.append(point)
        self.traj_pub.publish(trajectory)
        self.get_logger().info(f'Published {mode} arm trajectory with {len(trajectory.points)} points.')


def main():
    rclpy.init()
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
