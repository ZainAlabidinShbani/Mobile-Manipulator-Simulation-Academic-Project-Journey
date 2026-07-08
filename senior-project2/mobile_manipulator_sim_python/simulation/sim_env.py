"""
simulation/sim_env.py
----------------------
The simulation backend. Two implementations are provided behind a common
interface (`SimulationEnv`):

1. `PyBulletSimEnv`  - real 3D rigid-body simulation using PyBullet. The robot
   (mobile base + 6R arm + 2-finger gripper) and a target object are built
   procedurally from primitive shapes, so no external URDF/mesh files are
   required.
2. `DummySimEnv`     - a lightweight, dependency-free kinematic simulator used
   automatically if PyBullet is not installed (or fails to initialize, e.g. on
   a headless machine without OpenGL). It keeps the same API so the rest of
   the application (control, perception, tasks, GUI, logging) works
   unmodified regardless of backend.

`create_simulation()` is the factory function the rest of the app should use.
"""

import numpy as np
import config
from control import kinematics

try:
    import pybullet as p
    import pybullet_data
    PYBULLET_AVAILABLE = True
except Exception:  # pragma: no cover - environment without pybullet
    PYBULLET_AVAILABLE = False


class SimState:
    """Plain data container describing the current world state."""
    def __init__(self):
        self.base_pose = np.array([0.0, 0.0, 0.0])      # x, y, yaw
        self.joint_positions = config.ARM_HOME_CONFIG.copy()
        self.gripper_width = config.GRIPPER_OPEN_WIDTH
        self.object_position = np.array([1.0, 0.3, 0.05])
        self.object_grasped = False
        self.camera_image = None  # HxWx3 uint8


# ----------------------------------------------------------------------------
# PyBullet backend
# ----------------------------------------------------------------------------
class PyBulletSimEnv:
    def __init__(self, use_gui: bool = True):
        self.state = SimState()
        self._client = p.connect(p.GUI if use_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(config.SIM_TIMESTEP)
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        self._build_base_and_arm()
        self._build_object()
        self._grasp_constraint = None

    # -- construction -----------------------------------------------------
    def _build_base_and_arm(self):
        """Procedurally build a mobile base + 6R arm + 2-finger gripper using
        PyBullet's multi-body primitive creation (no URDF files needed)."""
        base_half_extents = [0.25, 0.2, 0.08]
        base_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=base_half_extents)
        base_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=base_half_extents,
                                        rgbaColor=[0.2, 0.3, 0.8, 1])

        link_masses, link_col, link_vis = [], [], []
        link_pos, link_orn, link_inertial_pos, link_inertial_orn = [], [], [], []
        link_parent, link_joint_type, link_joint_axis = [], [], []

        L = config.ARM_LINK_LENGTHS
        for i in range(config.ARM_NUM_JOINTS):
            length = max(L[min(i, len(L) - 1)], 0.05)
            radius = 0.035
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=length)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=length,
                                       rgbaColor=[0.8, 0.5, 0.1, 1])
            link_masses.append(0.5)
            link_col.append(col)
            link_vis.append(vis)
            link_pos.append([0, 0, length / 2 if i == 0 else length])
            link_orn.append([0, 0, 0, 1])
            link_inertial_pos.append([0, 0, 0])
            link_inertial_orn.append([0, 0, 0, 1])
            link_parent.append(0 if i == 0 else i)
            axis = [0, 0, 1] if i % 2 == 0 else [0, 1, 0]
            link_joint_axis.append(axis)
            link_joint_type.append(p.JOINT_REVOLUTE)

        # Two gripper finger links (prismatic, simplified as small boxes)
        finger_half = [0.01, 0.015, 0.04]
        for _ in range(2):
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=finger_half)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=finger_half,
                                       rgbaColor=[0.1, 0.1, 0.1, 1])
            link_masses.append(0.05)
            link_col.append(col)
            link_vis.append(vis)
            link_pos.append([0, 0, 0.05])
            link_orn.append([0, 0, 0, 1])
            link_inertial_pos.append([0, 0, 0])
            link_inertial_orn.append([0, 0, 0, 1])
            link_parent.append(config.ARM_NUM_JOINTS)
            link_joint_axis.append([0, 1, 0])
            link_joint_type.append(p.JOINT_PRISMATIC)

        self.robot_id = p.createMultiBody(
            baseMass=5.0,
            baseCollisionShapeIndex=base_col,
            baseVisualShapeIndex=base_vis,
            basePosition=[0, 0, 0.18],
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_col,
            linkVisualShapeIndices=link_vis,
            linkPositions=link_pos,
            linkOrientations=link_orn,
            linkInertialFramePositions=link_inertial_pos,
            linkInertialFrameOrientations=link_inertial_orn,
            linkParentIndices=link_parent,
            linkJointTypes=link_joint_type,
            linkJointAxis=link_joint_axis,
        )

        self.arm_joint_indices = list(range(config.ARM_NUM_JOINTS))
        self.finger_joint_indices = [config.ARM_NUM_JOINTS, config.ARM_NUM_JOINTS + 1]
        self.ee_link_index = config.ARM_NUM_JOINTS - 1

        for i, q in zip(self.arm_joint_indices, config.ARM_HOME_CONFIG):
            p.resetJointState(self.robot_id, i, q)

    def _build_object(self):
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.035)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.035, rgbaColor=[0.9, 0.1, 0.1, 1])
        self.object_id = p.createMultiBody(baseMass=0.1, baseCollisionShapeIndex=col,
                                            baseVisualShapeIndex=vis,
                                            basePosition=list(self.state.object_position))

    # -- API ----------------------------------------------------------------
    def step(self, base_cmd, arm_q_target, gripper_target, dt: float):
        v, w = base_cmd
        x, y, yaw = self.state.base_pose
        new_pose = np.array([
            x + v * np.cos(yaw) * dt,
            y + v * np.sin(yaw) * dt,
            yaw + w * dt,
        ])
        self.state.base_pose = new_pose
        quat = p.getQuaternionFromEuler([0, 0, new_pose[2]])
        p.resetBasePositionAndOrientation(self.robot_id, [new_pose[0], new_pose[1], 0.18], quat)

        for idx, q in zip(self.arm_joint_indices, arm_q_target):
            p.setJointMotorControl2(self.robot_id, idx, p.POSITION_CONTROL,
                                     targetPosition=q, force=200, maxVelocity=config.ARM_MAX_JOINT_VEL)
        half = gripper_target / 2.0
        p.setJointMotorControl2(self.robot_id, self.finger_joint_indices[0], p.POSITION_CONTROL,
                                 targetPosition=half, force=50)
        p.setJointMotorControl2(self.robot_id, self.finger_joint_indices[1], p.POSITION_CONTROL,
                                 targetPosition=-half, force=50)

        p.stepSimulation()

        joint_states = p.getJointStates(self.robot_id, self.arm_joint_indices)
        self.state.joint_positions = np.array([js[0] for js in joint_states])
        self.state.gripper_width = gripper_target

        if self.state.object_grasped and self._grasp_constraint is not None:
            pass  # constraint keeps object attached automatically
        else:
            obj_pos, _ = p.getBasePositionAndOrientation(self.object_id)
            self.state.object_position = np.array(obj_pos)

        self.state.camera_image = self._render_camera()
        return self.state

    def attach_object(self):
        ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
        ee_pos = ee_state[0]
        self._grasp_constraint = p.createConstraint(
            self.robot_id, self.ee_link_index, self.object_id, -1,
            p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0])
        self.state.object_grasped = True

    def release_object(self):
        if self._grasp_constraint is not None:
            p.removeConstraint(self._grasp_constraint)
            self._grasp_constraint = None
        self.state.object_grasped = False

    def teleport_object(self, position):
        p.resetBasePositionAndOrientation(self.object_id, list(position), [0, 0, 0, 1])
        self.state.object_position = np.array(position)

    def get_ee_pose(self):
        ee_state = p.getLinkState(self.robot_id, self.ee_link_index)
        return np.array(ee_state[0])

    def _render_camera(self):
        x, y, yaw = self.state.base_pose
        cam_eye = [x + 0.3 * np.cos(yaw), y + 0.3 * np.sin(yaw), 0.7]
        cam_target = [x + 1.3 * np.cos(yaw), y + 1.3 * np.sin(yaw), 0.3]
        view = p.computeViewMatrix(cam_eye, cam_target, [0, 0, 1])
        proj = p.computeProjectionMatrixFOV(config.CAMERA_FOV, config.CAMERA_WIDTH / config.CAMERA_HEIGHT,
                                             0.05, 5.0)
        _, _, rgb, _, _ = p.getCameraImage(config.CAMERA_WIDTH, config.CAMERA_HEIGHT, view, proj,
                                            renderer=p.ER_TINY_RENDERER)
        img = np.reshape(rgb, (config.CAMERA_HEIGHT, config.CAMERA_WIDTH, 4))[:, :, :3].astype(np.uint8)
        return img

    def reset(self):
        self.state = SimState()
        quat = p.getQuaternionFromEuler([0, 0, 0])
        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0.18], quat)
        for i, q in zip(self.arm_joint_indices, config.ARM_HOME_CONFIG):
            p.resetJointState(self.robot_id, i, q)
        self.release_object()
        self.teleport_object(self.state.object_position)

    def close(self):
        p.disconnect()


# ----------------------------------------------------------------------------
# Dummy (pure-kinematic) backend - no external dependencies besides NumPy
# ----------------------------------------------------------------------------
class DummySimEnv:
    """A minimal kinematic stand-in simulator used when PyBullet is
    unavailable. Integrates the base unicycle model and arm joints directly
    (no contact physics), and synthesizes a dummy camera image containing a
    colored circle representing the target object so the perception module
    still has something to detect."""

    def __init__(self, use_gui: bool = True):
        self.state = SimState()
        self._gui = use_gui

    def step(self, base_cmd, arm_q_target, gripper_target, dt: float):
        v, w = base_cmd
        self.state.base_pose = np.array([
            self.state.base_pose[0] + v * np.cos(self.state.base_pose[2]) * dt,
            self.state.base_pose[1] + v * np.sin(self.state.base_pose[2]) * dt,
            self.state.base_pose[2] + w * dt,
        ])

        q = self.state.joint_positions
        dq = np.clip(arm_q_target - q, -config.ARM_MAX_JOINT_VEL * dt, config.ARM_MAX_JOINT_VEL * dt)
        self.state.joint_positions = q + dq
        self.state.gripper_width = gripper_target

        if self.state.object_grasped:
            ee_pos = kinematics.end_effector_position(self.state.joint_positions, self.state.base_pose)
            self.state.object_position = ee_pos

        self.state.camera_image = self._synthesize_camera_image()
        return self.state

    def attach_object(self):
        self.state.object_grasped = True

    def release_object(self):
        self.state.object_grasped = False

    def teleport_object(self, position):
        self.state.object_position = np.array(position)

    def get_ee_pose(self):
        return kinematics.end_effector_position(self.state.joint_positions, self.state.base_pose)

    def _synthesize_camera_image(self):
        """Create a synthetic RGB image: a red disc whose screen position
        depends on the relative bearing/range of the object to the base, so
        that perception (dummy color detector or YOLO-on-synthetic-image)
        has plausible content to operate on."""
        h, w = config.CAMERA_HEIGHT, config.CAMERA_WIDTH
        img = np.full((h, w, 3), 30, dtype=np.uint8)  # dark background
        bx, by, byaw = self.state.base_pose
        ox, oy, oz = self.state.object_position
        dx, dy = ox - bx, oy - by
        rel_angle = np.arctan2(dy, dx) - byaw
        distance = np.hypot(dx, dy)

        # Project bearing to horizontal pixel coordinate, distance to apparent radius
        cx = int(w / 2 + (rel_angle / (config.CAMERA_FOV * np.pi / 180.0)) * w)
        cy = int(h / 2)
        radius = int(np.clip(25.0 / max(distance, 0.2), 4, 60))

        if -radius < cx < w + radius:
            import cv2
            cx_clamped = int(np.clip(cx, 0, w - 1))
            cv2.circle(img, (cx_clamped, cy), radius, (220, 40, 40), -1)  # RGB red
        return img

    def reset(self):
        self.state = SimState()

    def close(self):
        pass


# ----------------------------------------------------------------------------
# Factory
# ----------------------------------------------------------------------------
def create_simulation(use_gui: bool = True, prefer_pybullet: bool = True):
    """Instantiate the best available simulation backend."""
    if prefer_pybullet and PYBULLET_AVAILABLE:
        try:
            return PyBulletSimEnv(use_gui=use_gui)
        except Exception as exc:  # pragma: no cover
            print(f"[sim_env] PyBullet init failed ({exc}); falling back to DummySimEnv.")
    return DummySimEnv(use_gui=use_gui)
