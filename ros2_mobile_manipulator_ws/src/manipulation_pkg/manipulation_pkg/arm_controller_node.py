import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory


class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        self.declare_parameter('joint_names', ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4'])
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        self.goal_joints = [0.0, 0.0, 0.0, 0.0]
        self.progress = 1.0
        self.has_goal = False
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(JointTrajectory, '/arm_controller/joint_trajectory', self._traj_cb, 10)
        self.timer = self.create_timer(0.05, self._tick)

    def _traj_cb(self, msg):
        if not msg.points:
            return
        self.goal_joints = list(msg.points[-1].positions[:4])
        self.progress = 0.0
        self.has_goal = True
        self.get_logger().info('Arm trajectory accepted.')

    def _tick(self):
        if self.has_goal:
            self.progress = min(1.0, self.progress + 0.05)
            self.current_joints = [s + (g - s) * self.progress for s, g in zip(self.current_joints, self.goal_joints)]
            if self.progress >= 1.0:
                self.has_goal = False
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = list(self.get_parameter('joint_names').value)
        joint_state.position = list(self.current_joints)
        self.joint_pub.publish(joint_state)


def main():
    rclpy.init()
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
