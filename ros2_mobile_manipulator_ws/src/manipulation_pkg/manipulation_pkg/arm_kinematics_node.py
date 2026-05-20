import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mobile_manipulator_msgs.srv import SolveFK, SolveIK
from rclpy.node import Node


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    from geometry_msgs.msg import Quaternion
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def _pitch_from_quaternion(q):
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


class ArmKinematicsNode(Node):
    def __init__(self):
        super().__init__('arm_kinematics_node')
        self.declare_parameter('base_height', 0.32)
        self.declare_parameter('link_1', 0.28)
        self.declare_parameter('link_2', 0.24)
        self.declare_parameter('link_3', 0.14)
        self.ik_srv = self.create_service(SolveIK, 'solve_ik', self._solve_ik)
        self.fk_srv = self.create_service(SolveFK, 'solve_fk', self._solve_fk)

    def _solve_fk(self, request, response):
        joints = list(request.joint_positions)
        if len(joints) < 4:
            response.success = False
            response.message = 'Expected four joint positions.'
            return response
        q1, q2, q3, q4 = joints[:4]
        base_h = float(self.get_parameter('base_height').value)
        l1 = float(self.get_parameter('link_1').value)
        l2 = float(self.get_parameter('link_2').value)
        l3 = float(self.get_parameter('link_3').value)
        planar = l1 * math.cos(q2) + l2 * math.cos(q2 + q3) + l3 * math.cos(q2 + q3 + q4)
        x = math.cos(q1) * planar
        y = math.sin(q1) * planar
        z = base_h + l1 * math.sin(q2) + l2 * math.sin(q2 + q3) + l3 * math.sin(q2 + q3 + q4)
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = _quaternion_from_rpy(0.0, _normalize_angle(q2 + q3 + q4), q1)
        response.success = True
        response.pose = pose
        response.message = 'Forward kinematics solved.'
        return response

    def _solve_ik(self, request, response):
        pose = request.target_pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        q1 = math.atan2(y, x)
        r = math.hypot(x, y)
        l1 = float(self.get_parameter('link_1').value)
        l2 = float(self.get_parameter('link_2').value)
        l3 = float(self.get_parameter('link_3').value)
        base_h = float(self.get_parameter('base_height').value)
        pitch = _pitch_from_quaternion(pose.orientation)
        wrist_r = max(0.01, r - l3 * math.cos(pitch))
        wrist_z = z - base_h - l3 * math.sin(pitch)
        d = (wrist_r * wrist_r + wrist_z * wrist_z - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if d < -1.0 or d > 1.0:
            seed = list(request.seed_joint_positions)
            if len(seed) >= 4:
                response.success = False
                response.joint_positions = seed[:4]
                response.message = 'Target is outside the reachable workspace; returning seed configuration.'
            else:
                response.success = False
                response.joint_positions = [0.0, 0.0, 0.0, 0.0]
                response.message = 'Target is outside the reachable workspace.'
            return response
        q3 = math.acos(max(-1.0, min(1.0, d)))
        q2 = math.atan2(wrist_z, wrist_r) - math.atan2(l2 * math.sin(q3), l1 + l2 * math.cos(q3))
        q4 = pitch - q2 - q3
        response.success = True
        response.joint_positions = [q1, q2, q3, q4]
        response.message = 'Inverse kinematics solved.'
        return response


def main():
    rclpy.init()
    node = ArmKinematicsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
