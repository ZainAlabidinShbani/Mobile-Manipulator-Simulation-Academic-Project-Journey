"""
planning/trajectory.py
------------------------
Trajectory generation utilities for pick-and-place motions:
- Joint-space minimum-jerk trajectories (smooth start/stop, zero vel/acc at
  endpoints) for the arm.
- Cartesian waypoint trajectories (approach -> grasp -> lift -> transport ->
  place) built by chaining minimum-jerk segments and solving IK per waypoint.
"""

import numpy as np
import config
from control import kinematics


def minimum_jerk_scaling(t: float, duration: float) -> float:
    """Returns s(t) in [0, 1], the minimum-jerk time-scaling polynomial."""
    if duration <= 0:
        return 1.0
    tau = np.clip(t / duration, 0.0, 1.0)
    return 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5


def joint_space_trajectory(q_start: np.ndarray, q_end: np.ndarray, duration: float, dt: float):
    """Generate a list of joint configurations from q_start to q_end using a
    minimum-jerk time profile. Returns array of shape (N, num_joints)."""
    n_steps = max(int(np.ceil(duration / dt)), 1)
    traj = np.zeros((n_steps + 1, len(q_start)))
    for i in range(n_steps + 1):
        t = i * dt
        s = minimum_jerk_scaling(t, duration)
        traj[i] = q_start + s * (q_end - q_start)
    return traj


def cartesian_line_trajectory(p_start: np.ndarray, p_end: np.ndarray, duration: float, dt: float):
    """Generate a straight-line Cartesian trajectory (N, 3) with min-jerk timing."""
    n_steps = max(int(np.ceil(duration / dt)), 1)
    traj = np.zeros((n_steps + 1, 3))
    for i in range(n_steps + 1):
        t = i * dt
        s = minimum_jerk_scaling(t, duration)
        traj[i] = p_start + s * (p_end - p_start)
    return traj


def cartesian_traj_to_joint_traj(cart_traj: np.ndarray, base_pose: np.ndarray,
                                  q_seed: np.ndarray):
    """Convert a Cartesian position trajectory into a joint-space trajectory
    by solving IK sequentially, seeding each solve with the previous result
    (continuation method) for smooth, continuous joint motion."""
    joint_traj = np.zeros((len(cart_traj), len(q_seed)))
    q_guess = q_seed.copy()
    for i, p_target in enumerate(cart_traj):
        q_sol, converged, err = kinematics.inverse_kinematics(
            p_target, base_pose=base_pose, initial_guess=q_guess)
        if not converged:
            # keep best-effort solution; downstream controller will just track imperfectly
            pass
        joint_traj[i] = q_sol
        q_guess = q_sol
    return joint_traj


def build_pick_place_waypoints(object_pos: np.ndarray, place_pos: np.ndarray):
    """Return an ordered list of (name, cartesian_target) waypoints describing
    the full pick-and-place Cartesian path of the end effector."""
    approach_pick = object_pos + np.array([0, 0, config.APPROACH_HEIGHT])
    grasp_pos = object_pos + np.array([0, 0, config.GRASP_HEIGHT_OFFSET])
    lift_pos = approach_pick.copy()
    approach_place = place_pos + np.array([0, 0, config.APPROACH_HEIGHT])
    release_pos = place_pos + np.array([0, 0, config.GRASP_HEIGHT_OFFSET])
    retreat_pos = approach_place.copy()

    return [
        ("approach_object", approach_pick),
        ("descend_to_grasp", grasp_pos),
        ("lift_object", lift_pos),
        ("transport_to_place", approach_place),
        ("descend_to_place", release_pos),
        ("retreat", retreat_pos),
    ]
