"""
control/base_control.py
------------------------
Differential-drive controller for the mobile base. Implements a simple
go-to-pose controller (heading-then-distance) producing linear/angular
velocity commands, plus wheel velocity conversion utilities.
"""

import numpy as np
import config


def _wrap_to_pi(angle: float) -> float:
    return np.arctan2(np.sin(angle), np.cos(angle))


class BaseController:
    def __init__(self):
        self.target_pose = np.array([0.0, 0.0, 0.0])  # [x, y, yaw]
        self.k_rho = 1.0      # gain on distance error -> linear velocity
        self.k_alpha = 2.5    # gain on heading error -> angular velocity
        self.k_beta = -0.5    # gain on final orientation error

    def set_target(self, x: float, y: float, yaw: float = None):
        self.target_pose = np.array([x, y, yaw if yaw is not None else 0.0])
        self._yaw_specified = yaw is not None

    def compute_control(self, current_pose: np.ndarray):
        """
        current_pose: [x, y, yaw]
        Returns (v_linear, w_angular) command, plus distance/heading errors
        for monitoring.
        """
        x, y, yaw = current_pose
        tx, ty, tyaw = self.target_pose

        dx, dy = tx - x, ty - y
        rho = np.hypot(dx, dy)
        alpha = _wrap_to_pi(np.arctan2(dy, dx) - yaw)

        if rho < config.BASE_POS_TOLERANCE:
            # Close enough in position -> rotate in place to final yaw if requested
            beta_err = _wrap_to_pi(tyaw - yaw)
            v = 0.0
            w = np.clip(self.k_alpha * beta_err, -config.BASE_MAX_ANGULAR_VEL,
                        config.BASE_MAX_ANGULAR_VEL)
            return v, w, rho, beta_err

        v = self.k_rho * rho
        w = self.k_alpha * alpha

        # If the heading error is large, prioritize rotation over translation
        if abs(alpha) > np.pi / 2:
            v *= 0.2

        v = float(np.clip(v, -config.BASE_MAX_LINEAR_VEL, config.BASE_MAX_LINEAR_VEL))
        w = float(np.clip(w, -config.BASE_MAX_ANGULAR_VEL, config.BASE_MAX_ANGULAR_VEL))
        return v, w, rho, alpha

    def at_target(self, current_pose: np.ndarray) -> bool:
        x, y, yaw = current_pose
        tx, ty, tyaw = self.target_pose
        rho = np.hypot(tx - x, ty - y)
        yaw_err = abs(_wrap_to_pi(tyaw - yaw))
        return rho < config.BASE_POS_TOLERANCE and yaw_err < config.BASE_YAW_TOLERANCE

    @staticmethod
    def unicycle_to_wheel_speeds(v: float, w: float):
        """Convert (linear, angular) velocity to (left, right) wheel angular speeds."""
        r = config.BASE_WHEEL_RADIUS
        L = config.BASE_WHEEL_BASE
        w_left = (v - w * L / 2.0) / r
        w_right = (v + w * L / 2.0) / r
        return w_left, w_right

    @staticmethod
    def integrate_pose(pose: np.ndarray, v: float, w: float, dt: float) -> np.ndarray:
        """Simple unicycle kinematic integration (used by the dummy simulator)."""
        x, y, yaw = pose
        x += v * np.cos(yaw) * dt
        y += v * np.sin(yaw) * dt
        yaw = _wrap_to_pi(yaw + w * dt)
        return np.array([x, y, yaw])
