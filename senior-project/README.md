# Senior Project — Custom 3-DOF Mobile Manipulator

## Overview

This project presents the design, modeling, and control of a custom 3-degree-of-freedom (3-DOF) robotic arm mounted on a mobile base. It builds directly on the kinematic and control foundations established in the [semester project](../semester-project/README.md), with the objective of moving from a fixed, platform-based academic model (the KUKA youBot) toward an independently designed mechanical and control system.

The project serves as an intermediate design stage, bridging the foundational semester work and the full autonomous system developed in the [graduation project (Senior Project 2)](../senior-project2/README.md).

---

## 1. Objectives

- Design and evaluate a custom 3-DOF mobile manipulator concept
- Model arm and base behavior through MATLAB-based kinematic and dynamic analysis
- Design, implement, and compare multiple control strategies for trajectory tracking
- Validate mechanical feasibility through CAD design
- Document a design path suitable for future hardware implementation

---

## 2. System Overview

| Component        | Description                                                        |
|-------------------|---------------------------------------------------------------------|
| Manipulator       | Custom 3-DOF robotic arm                                            |
| Mechanical design | CAD-based mechanical modeling                                       |
| Analysis          | Forward/inverse kinematics, dynamics modeling (MATLAB)              |
| Control approach  | PID, LQR, and Fuzzy-PID — designed and benchmarked                  |
| Simulation        | MATLAB-based trajectory and control simulation                      |

---

## 3. Kinematics and Dynamics

Forward and inverse kinematics were derived for the 3-DOF arm, together with a dynamics model (mass, Coriolis, and gravity terms) used to evaluate joint torques under representative pick-and-place trajectories.

---

## 4. Control Strategies

Three control approaches were designed and evaluated under matched trajectory-tracking conditions, to compare tracking accuracy, robustness, and response quality:

- **PID control** — a baseline, model-independent approach
- **LQR (Linear-Quadratic Regulator)** — a model-based optimal control approach
- **Fuzzy-PID** — a hybrid approach combining fuzzy-logic gain adaptation with PID control, to improve robustness under varying operating conditions

Each controller was evaluated across multiple representative cases (nominal operation and operation with dynamic lag), with performance assessed through trajectory-tracking accuracy and joint-level response quality.

---

## 5. Mechanical Design

The mechanical structure of the arm and mobile base was modeled in CAD, with the design assessed for assembly feasibility and consistency with the kinematic model used in MATLAB.

---

## 6. Tools and Technologies

- **Analysis / Control design:** MATLAB
- **Mechanical design:** CAD

---

## 7. Repository Structure

```
senior-project/
├── matlab/
│   ├── CombinedSystem/    # Integrated kinematics, planning, and simulation
│   ├── Fuzzy/              # Fuzzy-PID controller design and evaluation
│   ├── LQR/                 # LQR controller design and evaluation
│   └── PID/                  # PID controller design and evaluation
├── cad/                        # Mechanical design files for the arm and mobile base
├── docs/                        # Report and technical documentation
└── media/                        # Figures and visual documentation
```

---

## 8. Evaluation Focus

This project is typically evaluated through:

- Kinematic consistency of the 3-DOF arm model
- Trajectory-tracking performance across PID, LQR, and Fuzzy-PID controllers
- Mechanical feasibility and assembly clarity in CAD
- Quality of design documentation and engineering rationale

---

## 9. Relationship to Other Repository Projects

- Compared with the [semester project](../semester-project/README.md), this project emphasizes original design decisions with a reduced arm complexity (3-DOF).
- Compared with [Senior Project 2](../senior-project2/README.md), this project served as an intermediate design and control study, ahead of the full autonomous, RL-controlled system developed for the graduation project.

---

## 10. Authors

**Zain Alabidin Shbani**
Bachelor's Degree in Robotics and Intelligent Systems Engineering

**Beilassan Hdewa**
Bachelor's Degree in Robotics and Intelligent Systems Engineering

**Lana Alwazzeh**
Bachelor's Degree in Robotics and Intelligent Systems Engineering
