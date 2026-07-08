"""
control/kinematics.py
----------------------
Forward and inverse kinematics for a generic N-DOF revolute arm (N is set by
config.ARM_NUM_JOINTS, e.g. 4-DOF) mounted on the mobile base. Implemented
with simple DH-style homogeneous transforms and a damped-least-squares
(Levenberg-Marquardt style) numerical IK solver so the math is transparent,
dependency-light (NumPy/SciPy only) and easy to extend.
"""

from dataclasses import dataclass
import numpy as np

import config


def _dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Standard Denavit-Hartenberg homogeneous transform matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1],
    ])


@dataclass
class DHParam:
    d: float
    a: float
    alpha: float


def _build_dh_table(num_joints: int, link_lengths: np.ndarray):
    """
    Build a generic N-DOF DH table from a list of link lengths:
      - joint 0   : waist/yaw joint, d = L[0], alpha = +90 deg (lifts the chain
                    out of the base plane so subsequent joints behave like a
                    shoulder/elbow pitching arm).
      - middle    : planar pitch joints (elbow-like), a = L[i], alpha = 0.
      - last joint: wrist joint, d = L[-1] (places the gripper tip), alpha = 0.

    This generalizes cleanly to any ARM_NUM_JOINTS (e.g. 4-DOF or 6-DOF arms)
    just by changing config.ARM_NUM_JOINTS / config.ARM_LINK_LENGTHS.
    """
    L = link_lengths
    table = [DHParam(d=L[0], a=0.0, alpha=np.pi / 2)]
    for i in range(1, num_joints - 1):
        table.append(DHParam(d=0.0, a=L[i], alpha=0.0))
    if num_joints > 1:
        table.append(DHParam(d=L[-1], a=0.0, alpha=0.0))
    return table


# DH table is generated dynamically from config so the arm's DOF count is a
# single source of truth (config.ARM_NUM_JOINTS / config.ARM_LINK_LENGTHS).
L = config.ARM_LINK_LENGTHS
DH_TABLE = _build_dh_table(config.ARM_NUM_JOINTS, L)


def forward_kinematics(joint_angles: np.ndarray, base_pose: np.ndarray = None) -> np.ndarray:
    """
    Compute the end-effector homogeneous transform (4x4) in the WORLD frame
    given joint_angles (rad, shape (6,)) and the mobile base pose
    base_pose = [x, y, yaw] (defaults to identity at the origin).

    Returns: 4x4 numpy array (SE(3) transform of the gripper tip in world frame).
    """
    if base_pose is None:
        base_pose = np.array([0.0, 0.0, 0.0])

    x, y, yaw = base_pose
    T = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, x],
        [np.sin(yaw),  np.cos(yaw), 0, y],
        [0,            0,           1, 0.35],  # arm base mounted 0.35 m above base
        [0,            0,           0, 1],
    ])

    for theta, dh in zip(joint_angles, DH_TABLE):
        T = T @ _dh_transform(theta, dh.d, dh.a, dh.alpha)

    return T


def forward_kinematics_all_links(joint_angles: np.ndarray, base_pose: np.ndarray = None):
    """Return list of 4x4 transforms for every joint frame (for visualization)."""
    if base_pose is None:
        base_pose = np.array([0.0, 0.0, 0.0])
    x, y, yaw = base_pose
    T = np.array([
        [np.cos(yaw), -np.sin(yaw), 0, x],
        [np.sin(yaw),  np.cos(yaw), 0, y],
        [0,            0,           1, 0.35],
        [0,            0,           0, 1],
    ])
    transforms = [T.copy()]
    for theta, dh in zip(joint_angles, DH_TABLE):
        T = T @ _dh_transform(theta, dh.d, dh.a, dh.alpha)
        transforms.append(T.copy())
    return transforms


def _numerical_jacobian(joint_angles: np.ndarray, base_pose: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Finite-difference 6xN Jacobian (position only, 3xN) of the end effector."""
    n = len(joint_angles)
    J = np.zeros((3, n))
    p0 = forward_kinematics(joint_angles, base_pose)[:3, 3]
    for i in range(n):
        dq = np.zeros(n)
        dq[i] = eps
        p1 = forward_kinematics(joint_angles + dq, base_pose)[:3, 3]
        J[:, i] = (p1 - p0) / eps
    return J


def inverse_kinematics(target_pos: np.ndarray,
                        base_pose: np.ndarray = None,
                        initial_guess: np.ndarray = None,
                        max_iters: int = None,
                        tol: float = None) -> tuple:
    """
    Damped least-squares numerical IK solving for the joint angles that place
    the end effector at `target_pos` (3,) in WORLD coordinates, given the
    current mobile base pose.

    Returns: (joint_angles (6,), converged: bool, final_error: float)
    """
    max_iters = max_iters or config.IK_MAX_ITERS
    tol = tol or config.IK_TOLERANCE
    q = np.array(initial_guess, dtype=float) if initial_guess is not None \
        else config.ARM_HOME_CONFIG.copy()

    lam = config.IK_DAMPING
    error_norm = np.inf

    for _ in range(max_iters):
        ee_pos = forward_kinematics(q, base_pose)[:3, 3]
        err = target_pos - ee_pos
        error_norm = np.linalg.norm(err)
        if error_norm < tol:
            return _clip_to_limits(q), True, error_norm

        J = _numerical_jacobian(q, base_pose)
        JJt = J @ J.T + (lam ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(JJt, err)
        q = q + dq
        q = _clip_to_limits(q)

    return q, False, error_norm


def _clip_to_limits(q: np.ndarray) -> np.ndarray:
    return np.clip(q, config.ARM_JOINT_LOWER, config.ARM_JOINT_UPPER)


def end_effector_position(joint_angles: np.ndarray, base_pose: np.ndarray = None) -> np.ndarray:
    """Convenience helper returning just the (x, y, z) position."""
    return forward_kinematics(joint_angles, base_pose)[:3, 3]
