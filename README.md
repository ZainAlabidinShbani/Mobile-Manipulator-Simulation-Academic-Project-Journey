# Mobile Manipulator Simulation — Academic Project Journey

This repository documents my academic and research progression in mobile manipulator design, modeling, and control — from coursework foundations to an independently designed graduation project. It contains three projects, presented in chronological and technical progression from foundational to advanced.

---

## 📁 Projects (in reading order)

### 1. [Semester Project — KUKA youBot Mobile Manipulator](./semester-project/README.md)
> 📂 `semester-project/`

A simulation and control framework built around the KUKA youBot, integrating its omnidirectional mecanum-wheeled base with a 5-degree-of-freedom (5-DOF) robotic arm. Establishes the kinematic, dynamic, and control foundations used throughout the later projects.

- 5-DOF arm on a mecanum-wheeled omnidirectional base
- Trajectory generation and manipulability analysis
- Observer-based state feedback control
- MATLAB + ROS Noetic + CoppeliaSim integration

---

### 2. [Senior Project — Custom 3-DOF Mobile Manipulator](./senior-project/README.md)
> 📂 `senior-project/`

An intermediate, independently designed 3-DOF robotic arm on a mobile base, developed to move beyond a fixed academic platform toward original mechanical and control design. Serves as a bridge between the semester project and the graduation project.

- Custom 3-DOF arm kinematics and dynamics, modeled in MATLAB
- Multi-controller design and evaluation: PID, LQR, and Fuzzy-PID
- CAD-based mechanical design and trajectory-tracking validation

---

### 3. [Senior Project 2 — Autonomous Mobile Manipulator (Graduation Project)](./senior-project2/README.md)
> 📂 `senior-project2/`

My graduation project for the Bachelor's degree in Robotics and Intelligent Systems Engineering. Presents the design of a prototype autonomous mobile manipulator for pick-and-place operations in static indoor environments: a four-wheel skid-steer base with a custom 4-DOF parallelogram-linkage arm, controlled through reinforcement learning (PPO and SAC) and supported by a YOLO-based perception pipeline.

- 4-DOF parallelogram-linkage arm on a four-wheel skid-steer base, modeled in SolidWorks
- Static and dynamic stability validated via Zero Moment Point (ZMP) analysis (safety factor > 1.5)
- Reinforcement learning control (PPO vs. SAC) for autonomous pick-and-place
- YOLO-based perception pipeline (RGB-D and low-cost monocular alternatives)
- Simulation-first ROS 2 Humble workspace and a standalone Python simulation stack
- Physical prototype with electronics integration (Arduino Mega, motor drivers, sensors)

---

## 🔍 Quick Comparison

| Feature      | Semester Project        | Senior Project              | Senior Project 2 (Graduation)         |
|--------------|--------------------------|-------------------------------|-----------------------------------------|
| Robot        | KUKA youBot               | Custom-designed manipulator   | Custom-designed autonomous manipulator  |
| Arm DOF      | 5                          | 3                              | 4 (parallelogram linkage)               |
| Base         | Mecanum-wheel (omni)       | Mobile base                   | Four-wheel skid-steer                   |
| Control      | Observer-based feedback    | PID, LQR, Fuzzy-PID            | Reinforcement learning (PPO, SAC)       |
| Perception   | —                          | —                              | YOLO-based detection (RGB-D / monocular)|
| Stability    | —                          | —                              | Zero Moment Point (ZMP) analysis        |
| CAD          | —                          | Included                       | SolidWorks, full electromechanical BOM  |
| Tools        | MATLAB, ROS Noetic, CoppeliaSim | MATLAB                   | MATLAB, Python, ROS 2 Humble, CoppeliaSim, Gazebo |

---

## 🚀 Getting Started

Navigate to the project subfolder of your choice and follow its `README.md`:

```bash
# Foundational project (5-DOF KUKA youBot)
cd semester-project/

# Intermediate project (custom 3-DOF arm)
cd senior-project/

# Graduation project (autonomous 4-DOF mobile manipulator)
cd senior-project2/
```

---

## 🧰 Technologies

MATLAB · Python · SolidWorks · CoppeliaSim · Gazebo · ROS Noetic · ROS 2 Humble · YOLO (Ultralytics) · Reinforcement Learning (PPO, SAC) · PyBullet · Arduino

---

## 👤 Author

**Zain Alabidin Shbani**
Bachelor's Degree in Robotics and Intelligent Systems Engineering
