# Semester Project: KUKA youBot Mobile Manipulator

> Academic project | MATLAB, ROS Noetic, and CoppeliaSim

## Overview

This project presents a complete simulation and control framework for a KUKA youBot-style mobile manipulator. The system combines an omnidirectional mecanum-wheeled base with a 5-degree-of-freedom (5-DOF) robotic arm, and integrates **MATLAB**, **ROS Noetic**, and **CoppeliaSim** for kinematic analysis, control design, and visualization. It establishes the technical foundation later built upon in the [senior project](../senior-project/README.md) and the [graduation project](../senior-project2/README.md).

---

## Objectives

- Build a complete mobile manipulator simulation baseline
- Validate arm and base kinematics in coordinated tasks
- Implement and evaluate observer-based state feedback control
- Connect mathematical modeling to 3D simulation and ROS visualization

---

## System overview

| Component        | Description                                                        |
|-------------------|---------------------------------------------------------------------|
| Mobile base       | Mecanum-wheeled omnidirectional platform                            |
| Manipulator       | 5-DOF robotic arm                                                    |
| Control approach  | Observer-based state feedback control                               |
| Analysis          | Trajectory planning, manipulability analysis                        |
| Simulation        | CoppeliaSim, MATLAB                                                  |
| Middleware        | ROS Noetic (URDF/Xacro, RViz, TF)                                    |

---

## Academic focus

The project was developed to study core robotics topics within a single, connected workflow:

- Mobile manipulator kinematics and dynamics
- Trajectory generation and tracking
- Observer-based state feedback control
- Multi-tool simulation and validation (MATLAB ↔ ROS ↔ CoppeliaSim)

---

## Kinematics, control, and analysis

Forward and inverse kinematics were derived for the combined base–arm system, and an **observer-based state feedback controller** was designed to regulate joint and base states where direct measurement was unavailable or noisy. **Trajectory planning** and **manipulability analysis** were carried out across the arm's workspace to characterize dexterity and identify configurations prone to kinematic singularities.

---

## Tools and environment

- **MATLAB** — modeling, control algorithms, and analysis
- **CoppeliaSim** — 3D scenario testing
- **ROS Noetic** (optional) — robot description, TF, and RViz validation

---

## Repository structure

```
semester-project/
├── matlab/               # Core modeling and control scripts (entry: MobileManipulatorMain.m)
├── kuka_youbot/            # ROS Noetic package (URDF/Xacro, launch files, RViz, runtime nodes)
├── Coppeliasim_Scenes/       # CoppeliaSim scenes for 3D simulation
├── csv files/                  # Recorded trajectories, states, and motion data
├── docs/                         # Report and technical documentation
└── media/                          # Figures and project visuals
```

---

## How to use

### Prerequisites

- MATLAB with the required toolboxes for the included scripts
- CoppeliaSim for 3D scene execution
- ROS Noetic on Linux for the optional ROS integration

The MATLAB and CoppeliaSim workflows can be used independently of ROS.

### 7.1 MATLAB Simulation

```matlab
% Set working directory to semester-project/matlab/, then run:
MobileManipulatorMain
```

### 7.2 CoppeliaSim Scene

1. Open CoppeliaSim.
2. Load `Coppeliasim_Scenes/Scene6_youBot_cube.ttt`.
3. Use matching model parameters with MATLAB for consistent results.

### 7.3 ROS Noetic (optional)

Intended for a Linux ROS Noetic environment with a configured catkin workspace:

```bash
catkin_make
source devel/setup.bash
roslaunch kuka_youbot full_robot.launch
```

---

## Evaluation focus

- Path-tracking quality of the arm and base
- Stability and smoothness of control responses
- Manipulability trends across trajectories
- Consistency between MATLAB and simulator outputs

---

## Outputs and analysis

The CSV files in `csv files/` can be replayed and analyzed in MATLAB for base and arm motion behavior, tracking performance, and end-effector trajectory quality.

---

## Media

See `media/` for supporting figures, including frame-assignment diagrams, manipulability results, and wheel-behavior plots.

---

## Authors

**Zain Alabidin Shbani**
Bachelor's Degree in Robotics and Intelligent Systems Engineering

**Beilassan Hdewa**
Bachelor's Degree in Robotics and Intelligent Systems Engineering

**Lana Alwazzeh**
Bachelor's Degree in Robotics and Intelligent Systems Engineering
