"""
config.py
---------
Central configuration constants for the mobile manipulator simulation.
Keeping all tunable parameters in one place makes the rest of the codebase
easier to read and to extend.
"""

import numpy as np

# ----------------------------------------------------------------------------
# Simulation
# ----------------------------------------------------------------------------
SIM_TIMESTEP = 1.0 / 240.0          # PyBullet physics timestep [s]
CONTROL_DT = 1.0 / 30.0             # Control / GUI update period [s]
USE_GUI_PHYSICS_RENDER = True       # Open PyBullet's own debug GUI window too

# ----------------------------------------------------------------------------
# Mobile base (differential drive)
# ----------------------------------------------------------------------------
BASE_WHEEL_RADIUS = 0.08            # [m]
BASE_WHEEL_BASE = 0.45              # distance between wheels [m]
BASE_MAX_LINEAR_VEL = 0.6           # [m/s]
BASE_MAX_ANGULAR_VEL = 1.5          # [rad/s]
BASE_POS_TOLERANCE = 0.05           # [m]
BASE_YAW_TOLERANCE = 0.05           # [rad]

# ----------------------------------------------------------------------------
# Arm (generic N-DOF revolute manipulator, simple DH-style geometry)
# ----------------------------------------------------------------------------
# Single source of truth for the arm's degrees of freedom. Change this (and
# the arrays below to match length) to use a 4-DOF, 5-DOF, 6-DOF, etc. arm.
ARM_NUM_JOINTS = 4

# Link lengths along the kinematic chain [m].
# [0] = waist height offset, [1..N-2] = upper-arm/forearm link lengths,
# [-1] = wrist-to-gripper-tip offset.
ARM_LINK_LENGTHS = np.array([0.12, 0.30, 0.25, 0.12])

# Joint limits [rad] -- one pair per joint (length must match ARM_NUM_JOINTS)
ARM_JOINT_LOWER = np.array([-np.pi, -np.pi / 2, -np.pi / 2, -np.pi])
ARM_JOINT_UPPER = np.array([np.pi, np.pi / 2, np.pi / 2, np.pi])

ARM_MAX_JOINT_VEL = 2.0             # [rad/s] per joint
ARM_HOME_CONFIG = np.array([0.0, -0.4, 0.6, 0.0])

# Gripper
GRIPPER_OPEN_WIDTH = 0.08           # [m]
GRIPPER_CLOSED_WIDTH = 0.0          # [m]
GRIPPER_MAX_VEL = 0.05              # [m/s]

# IK solver
IK_MAX_ITERS = 150
IK_TOLERANCE = 1e-3
IK_DAMPING = 0.05

# ----------------------------------------------------------------------------
# Perception
# ----------------------------------------------------------------------------
CAMERA_WIDTH = 320
CAMERA_HEIGHT = 240
CAMERA_FOV = 60.0
YOLO_MODEL_PATH = "yolov8n.pt"
YOLO_CONF_THRESHOLD = 0.35
TARGET_CLASS_NAMES = ["sports ball", "cup", "bottle", "box", "cube"]  # accepted classes

# ----------------------------------------------------------------------------
# Trajectory generation
# ----------------------------------------------------------------------------
TRAJ_DEFAULT_DURATION = 2.5         # [s]
TRAJ_SAMPLE_DT = CONTROL_DT

# Pick-and-place geometry offsets
APPROACH_HEIGHT = 0.15              # [m] above object before descending
GRASP_HEIGHT_OFFSET = 0.02          # [m] gripper tip offset above object center
PLACE_LOCATION = np.array([1.2, 0.6, 0.05])  # default world-frame place target

# ----------------------------------------------------------------------------
# Logging
# ----------------------------------------------------------------------------
LOG_DIR = "data"
LOG_FILENAME_FMT = "sim_log_%Y%m%d_%H%M%S.csv"
LOG_FLUSH_EVERY_N_ROWS = 20
