# Semester Project — KUKA youBot Mobile Manipulator Simulation Framework

## Overview

This project presents a complete simulation and control framework for a KUKA youBot-style mobile manipulator. The system combines an omnidirectional mecanum-wheeled base with a 5-degree-of-freedom (5-DOF) robotic arm, and integrates **MATLAB**, **ROS Noetic**, and **CoppeliaSim** for kinematic analysis, control design, and visualization. It establishes the technical foundation later built upon in the [senior project](../senior-project/README.md) and the [graduation project](../senior-project2/README.md).

---

## 1. Objectives

- Build a complete mobile manipulator simulation baseline
- Validate arm and base kinematics in coordinated tasks
- Implement and evaluate observer-based state feedback control
- Connect mathematical modeling to 3D simulation and ROS visualization

---

## 2. System Overview

| Component        | Description                                                        |
|-------------------|---------------------------------------------------------------------|
| Mobile base       | Mecanum-wheeled omnidirectional platform                            |
| Manipulator       | 5-DOF robotic arm                                                    |
| Control approach  | Observer-based state feedback control                               |
| Analysis          | Trajectory planning, manipulability analysis                        |
| Simulation        | CoppeliaSim, MATLAB                                                  |
| Middleware        | ROS Noetic (URDF/Xacro, RViz, TF)                                    |

---

## 3. Academic Focus

The project was developed to study core robotics topics within a single, connected workflow:

- Mobile manipulator kinematics and dynamics
- Trajectory generation and tracking
- Observer-based state feedback control
- Multi-tool simulation and validation (MATLAB ↔ ROS ↔ CoppeliaSim)

---

## 4. Kinematics, Control, and Analysis

Forward and inverse kinematics were derived for the combined base–arm system, and an **observer-based state feedback controller** was designed to regulate joint and base states where direct measurement was unavailable or noisy. **Trajectory planning** and **manipulability analysis** were carried out across the arm's workspace to characterize dexterity and identify configurations prone to kinematic singularities.

---

## 5. Tools and Environment

- **MATLAB** — modeling, control algorithms, and analysis
- **CoppeliaSim** — 3D scenario testing
- **ROS Noetic** (optional) — robot description, TF, and RViz validation

---

## 6. Repository Structure

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

## 7. How to Use

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

## 8. Evaluation Focus

- Path-tracking quality of the arm and base
- Stability and smoothness of control responses
- Manipulability trends across trajectories
- Consistency between MATLAB and simulator outputs

---

## 9. Outputs and Analysis

The CSV files in `csv files/` can be replayed and analyzed in MATLAB for base and arm motion behavior, tracking performance, and end-effector trajectory quality.

---

## 10. Media

See `media/` for supporting figures, including frame-assignment diagrams, manipulability results, and wheel-behavior plots.

---

## 11. Author

**Zain Alabidin Shbani**
Bachelor's Degree in Robotics and Intelligent Systems Engineering
