# Semester Project - KUKA youBot Mobile Manipulator Simulation Framework

This semester project presents a full simulation and control framework for a KUKA youBot-style mobile manipulator. The system combines an omnidirectional mecanum base with a 5-DOF robotic arm, and integrates MATLAB, ROS Noetic, and CoppeliaSim for analysis, control design, and visualization.

## Objectives

- build a complete mobile manipulator simulation baseline
- validate arm and base kinematics in coordinated tasks
- implement and evaluate observer-based feedback control
- connect mathematical modeling to 3D simulation and ROS visualization

## Academic focus

The project was developed to study core robotics topics in a single, connected workflow:

- mobile manipulator kinematics and dynamics
- trajectory generation and tracking
- observer-based state feedback control
- multi-tool simulation and validation

## Tools and environment

- MATLAB for modeling, control algorithms, and analysis
- CoppeliaSim for 3D scenario testing
- ROS Noetic (optional) for robot description, TF, and RViz validation

## Repository layout

- `matlab/` - Core modeling and control scripts. Main entry: `MobileManipulatorMain.m`.
- `kuka_youbot/` - ROS Noetic package (URDF/Xacro, launch files, RViz setup, runtime nodes).
- `Coppeliasim_Scenes/` - CoppeliaSim scenes for 3D simulation.
- `csv files/` - Recorded trajectories, states, and motion data.
- `docs/` - Reports and technical documentation.
- `media/` - Figures and project visuals.

## Technical highlights

- 5-DOF arm and omnidirectional mobile base modeling
- Integrated MATLAB + ROS + CoppeliaSim workflow
- End-effector path visualization and TF-based monitoring
- Manipulability and trajectory analysis using exported data
- Structured launch setup for system-level ROS testing

## How to use

### 1) MATLAB simulation

1. Open MATLAB.
2. Set the working directory to `semester-project/matlab/`.
3. Run `MobileManipulatorMain.m`.

### 2) CoppeliaSim scene

1. Open CoppeliaSim.
2. Load `Coppeliasim_Scenes/Scene6_youBot_cube.ttt`.
3. Use matching model parameters with MATLAB for consistent results.

### 3) ROS Noetic run (optional)

This part is intended for a Linux ROS Noetic environment with a configured catkin workspace.

```bash
catkin_make
source devel/setup.bash
roslaunch kuka_youbot full_robot.launch
```

## What to evaluate

Typical evaluation points for this project include:

- path tracking quality of the arm and base
- stability and smoothness of control responses
- manipulability trends across trajectories
- consistency between MATLAB and simulator outputs

## Outputs and analysis

The CSV files can be replayed and analyzed in MATLAB for:

- base and arm motion behavior
- tracking performance
- end-effector trajectory quality

## Media

Project visuals are provided in `media/`, including assembly renders, frame assignment diagrams, manipulability results, and wheel behavior plots.
