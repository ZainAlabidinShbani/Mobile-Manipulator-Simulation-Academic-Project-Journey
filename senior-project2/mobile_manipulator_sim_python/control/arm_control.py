"""
control/arm_control.py
-----------------------
Joint-space controller for the 4-DOF arm. Wraps a simple PD-style velocity
controller that drives the current joint configuration towards a commanded
setpoint, respecting velocity limits. This controller is simulator-agnostic:
it produces joint velocity commands that the simulation backend applies.
"""

import numpy as np
import config


class ArmController:
    def __init__(self):
        self.q_cmd = config.ARM_HOME_CONFIG.copy()      # commanded joint targets
        self.q_current = config.ARM_HOME_CONFIG.copy()  # last known measured joints
        self.kp = 4.0                                    # position gain -> velocity
        self.gripper_width_cmd = config.GRIPPER_OPEN_WIDTH
        self.gripper_width_current = config.GRIPPER_OPEN_WIDTH

    def set_joint_target(self, q_target: np.ndarray):
        self.q_cmd = np.clip(q_target, config.ARM_JOINT_LOWER, config.ARM_JOINT_UPPER)

    def set_gripper(self, width: float):
        self.gripper_width_cmd = float(np.clip(width, config.GRIPPER_CLOSED_WIDTH,
                                                config.GRIPPER_OPEN_WIDTH))

    def update(self, q_measured: np.ndarray, gripper_measured: float, dt: float):
        """
        Compute joint velocity commands (rad/s) for one control step using a
        simple proportional law clamped to the max joint velocity, then
        integrate to emulate the commanded next joint state (used by the
        dummy simulator backend; PyBullet backend uses POSITION_CONTROL
        directly with q_cmd).
        """
        self.q_current = q_measured
        self.gripper_width_current = gripper_measured

        error = self.q_cmd - self.q_current
        vel = np.clip(self.kp * error, -config.ARM_MAX_JOINT_VEL, config.ARM_MAX_JOINT_VEL)
        q_next = self.q_current + vel * dt

        grip_error = self.gripper_width_cmd - self.gripper_width_current
        grip_vel = np.clip(grip_error / max(dt, 1e-6), -config.GRIPPER_MAX_VEL, config.GRIPPER_MAX_VEL)
        grip_next = self.gripper_width_current + grip_vel * dt

        return q_next, grip_next, vel

    def at_target(self, tol: float = 0.02) -> bool:
        return np.all(np.abs(self.q_cmd - self.q_current) < tol)

    def gripper_at_target(self, tol: float = 0.005) -> bool:
        return abs(self.gripper_width_cmd - self.gripper_width_current) < tol
