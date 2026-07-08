"""
tasks/task_manager.py
-----------------------
Finite-state-machine task logic implementing the full pick-and-place mission:

    DETECT -> MOVE_BASE -> ALIGN_ARM -> GRASP -> TRANSPORT -> PLACE -> DONE

The TaskManager owns the high-level decision-making and delegates motion
execution to BaseController / ArmController, and target perception to
ObjectDetector. It is driven by calling `.update()` once per control tick.
"""

from enum import Enum, auto
import numpy as np

import config
from control import kinematics
from planning import trajectory


class TaskState(Enum):
    IDLE = auto()
    DETECT = auto()
    MOVE_BASE = auto()
    ALIGN_ARM = auto()
    GRASP = auto()
    TRANSPORT = auto()
    PLACE = auto()
    DONE = auto()
    ERROR = auto()


class TaskManager:
    def __init__(self, base_controller, arm_controller, detector):
        self.base_controller = base_controller
        self.arm_controller = arm_controller
        self.detector = detector

        self.state = TaskState.IDLE
        self.object_world_pos = None
        self.place_world_pos = config.PLACE_LOCATION.copy()
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.last_error = 0.0
        self.status_message = "Idle"
        self._arm_traj = None
        self._arm_traj_idx = 0
        self._arm_traj_target_pos = np.zeros(3)

    # -- public control -------------------------------------------------
    def start(self):
        self.state = TaskState.DETECT
        self.status_message = "Searching for target object..."

    def stop(self):
        self.state = TaskState.IDLE
        self.status_message = "Stopped"

    def reset(self):
        self.__init__(self.base_controller, self.arm_controller, self.detector)

    # -- main FSM update --------------------------------------------------
    def update(self, sim_state, dt: float):
        """Advance the task state machine by one tick. Returns dict of
        commands: {base_target: (x,y,yaw) or None, arm_target: q or None,
        gripper_target: width or None}."""

        cmd = {"base_target": None, "arm_target": None, "gripper_target": None}

        if self.state == TaskState.IDLE:
            pass

        elif self.state == TaskState.DETECT:
            det = self.detector.best_target(sim_state.camera_image)
            if det is not None:
                bx, by, byaw = sim_state.base_pose
                angle = byaw + det.bearing_rad
                self.object_world_pos = np.array([
                    bx + det.est_range_m * np.cos(angle),
                    by + det.est_range_m * np.sin(angle),
                    0.05,
                ])
                self.status_message = f"Object detected ({det.class_name}, conf={det.confidence:.2f})"
                self.state = TaskState.MOVE_BASE
            else:
                self.status_message = "No object detected yet..."

        elif self.state == TaskState.MOVE_BASE:
            target_x = self.object_world_pos[0] - 0.45
            target_y = self.object_world_pos[1]
            yaw = np.arctan2(self.object_world_pos[1] - target_y, self.object_world_pos[0] - target_x)
            self.base_controller.set_target(target_x, target_y, yaw)
            cmd["base_target"] = (target_x, target_y, yaw)
            v, w, rho, alpha = self.base_controller.compute_control(sim_state.base_pose)
            self.last_error = rho
            cmd["base_cmd"] = (v, w)
            self.status_message = f"Moving base to object (dist={rho:.2f} m)"
            if self.base_controller.at_target(sim_state.base_pose):
                self.status_message = "Base in position. Planning arm motion."
                self.waypoints = trajectory.build_pick_place_waypoints(
                    self.object_world_pos, self.place_world_pos)
                self.current_waypoint_idx = 0
                self._start_next_waypoint(sim_state)
                self.state = TaskState.ALIGN_ARM

        elif self.state == TaskState.ALIGN_ARM:
            cmd["base_cmd"] = (0.0, 0.0)
            done = self._advance_arm_trajectory(sim_state, cmd)
            if done and self.current_waypoint_idx == 2:  # finished descend_to_grasp
                self.state = TaskState.GRASP

        elif self.state == TaskState.GRASP:
            cmd["base_cmd"] = (0.0, 0.0)
            self.arm_controller.set_gripper(config.GRIPPER_CLOSED_WIDTH)
            cmd["gripper_target"] = config.GRIPPER_CLOSED_WIDTH
            if self.arm_controller.gripper_at_target():
                self.status_message = "Object grasped. Lifting."
                self.state = TaskState.TRANSPORT

        elif self.state == TaskState.TRANSPORT:
            cmd["base_cmd"] = (0.0, 0.0)
            done = self._advance_arm_trajectory(sim_state, cmd)
            if done and self.current_waypoint_idx == 4:  # finished transport_to_place
                self.state = TaskState.PLACE

        elif self.state == TaskState.PLACE:
            cmd["base_cmd"] = (0.0, 0.0)
            if self.current_waypoint_idx == 4:
                # still descending towards place position
                done = self._advance_arm_trajectory(sim_state, cmd)
                if done:
                    self.arm_controller.set_gripper(config.GRIPPER_OPEN_WIDTH)
                    cmd["gripper_target"] = config.GRIPPER_OPEN_WIDTH
                    if self.arm_controller.gripper_at_target():
                        self.status_message = "Object placed. Retreating."
            else:
                done = self._advance_arm_trajectory(sim_state, cmd)
                if done and self.current_waypoint_idx >= len(self.waypoints):
                    self.state = TaskState.DONE
                    self.status_message = "Pick-and-place task complete."

        elif self.state == TaskState.DONE:
            cmd["base_cmd"] = (0.0, 0.0)

        return cmd

    # -- helpers -----------------------------------------------------------
    def _start_next_waypoint(self, sim_state):
        if self.current_waypoint_idx >= len(self.waypoints):
            self._arm_traj = None
            return
        name, target_pos = self.waypoints[self.current_waypoint_idx]
        ee_pos = kinematics.end_effector_position(sim_state.joint_positions, sim_state.base_pose)
        cart_traj = trajectory.cartesian_line_trajectory(
            ee_pos, target_pos, config.TRAJ_DEFAULT_DURATION, config.TRAJ_SAMPLE_DT)
        self._arm_traj = trajectory.cartesian_traj_to_joint_traj(
            cart_traj, sim_state.base_pose, sim_state.joint_positions)
        self._arm_traj_idx = 0
        self._arm_traj_target_pos = target_pos.copy()
        self.status_message = f"Executing arm motion: {name}"

    def _advance_arm_trajectory(self, sim_state, cmd):
        """Step through the currently active joint trajectory. Returns True
        once the trajectory has been fully executed (and advances to next
        waypoint automatically)."""
        if self._arm_traj is None:
            self._start_next_waypoint(sim_state)
            if self._arm_traj is None:
                return True

        if self._arm_traj_idx < len(self._arm_traj):
            q_target = self._arm_traj[self._arm_traj_idx]
            self.arm_controller.set_joint_target(q_target)
            cmd["arm_target"] = q_target
            self._arm_traj_idx += 1
            self.last_error = float(np.linalg.norm(
                kinematics.end_effector_position(sim_state.joint_positions, sim_state.base_pose)
                - self._arm_traj_target_pos))
            return False
        else:
            # trajectory finished -> move to next waypoint
            self.current_waypoint_idx += 1
            self._arm_traj = None
            if self.current_waypoint_idx < len(self.waypoints):
                self._start_next_waypoint(sim_state)
            return True
