# Senior Project 2 — Autonomous Mobile Manipulator for Indoor Pick-and-Place (Graduation Project)

## Abstract

This project presents the design of a prototype autonomous mobile manipulator intended to perform pick-and-place operations in static indoor environments. The system consists of a four-wheel skid-steer mobile platform integrated with a 4-degree-of-freedom (4-DOF) parallelogram-linkage robotic arm. The mechanical design was modeled in SolidWorks, and both static and dynamic stability were assessed using Zero Moment Point (ZMP) theory; the system was found to operate safely across all tested configurations, with a safety factor greater than 1.5.

The control framework is built on reinforcement learning (RL). Two algorithms — Proximal Policy Optimization (PPO) and Soft Actor-Critic (SAC) — were designed, trained, and evaluated in a custom two-dimensional navigation environment. The RL agent observes a 14-dimensional state vector encoding joint angles, joint velocities, end-effector position, base pose, relative target displacement, and gripper state, and outputs a 5-dimensional continuous action vector governing arm joint velocities and base linear and angular velocities. An eight-component reward function, combining dense shaping signals with sparse milestone bonuses, was formulated to guide the agent through the reaching, grasping, transporting, and placing subtasks.

Experimental results show that SAC outperforms PPO in sample efficiency, final reward, success rate, and path quality, while PPO offers superior training stability during early exploration. A YOLO-based perception pipeline was additionally designed to support object detection and localization. Both RGB-D-based localization (full 3D perception) and Raspberry Pi Camera Module 3-based monocular localization (a low-cost implementation alternative) are discussed.

**Keywords:** Mobile Manipulator, Pick-and-Place, Reinforcement Learning, PPO, SAC, YOLO, Skid-Steer Robot, CoppeliaSim, Robotic Arm Kinematics, ZMP Stability Analysis

This project was developed as our graduation project for the Bachelor's degree in Robotics and Intelligent Systems Engineering, and is documented in full in the accompanying [report](./report) and [presentation](./presentation).

---

## 1. System Overview

| Component        | Description                                                        |
|-------------------|---------------------------------------------------------------------|
| Mobile base       | Four-wheel skid-steer platform                                     |
| Manipulator       | 4-DOF parallelogram-linkage robotic arm                            |
| CAD / Mechanical  | Designed and modeled in SolidWorks                                 |
| Stability method  | Zero Moment Point (ZMP) — static and dynamic analysis              |
| Control approach  | Reinforcement learning (PPO, SAC)                                  |
| Perception        | YOLO-based object detection and localization                        |
| Simulation        | CoppeliaSim, Gazebo, custom Python 2D RL training environment       |
| Middleware        | Simulation-first ROS 2 Humble workspace                            |
| Hardware          | Arduino Mega-based electronics and motor control                    |

---

## 2. Mechanical Design and Stability Analysis

The mobile manipulator was modeled in SolidWorks as a four-wheel skid-steer base carrying a 4-DOF arm built on a parallelogram-linkage configuration, chosen to maintain end-effector orientation through the arm's range of motion while simplifying the kinematic model relative to a fully serial chain.

Stability was evaluated using **Zero Moment Point (ZMP) theory**, applied to both static configurations (the arm fully extended, retracted, and at intermediate reach positions) and dynamic conditions (during base motion and arm articulation). The ZMP was verified to remain within the support polygon in all tested configurations, yielding a **safety factor greater than 1.5** throughout.

---

## 3. Reinforcement Learning Control Framework

### 3.1 Problem Formulation

Manipulation and navigation are jointly controlled through a single RL policy operating in a custom two-dimensional simulation environment.

- **State space (14 dimensions):** joint angles, joint velocities, end-effector position, base pose, relative target displacement, gripper state
- **Action space (5 dimensions, continuous):** arm joint velocities, base linear velocity, base angular velocity
- **Reward function (8 components):** dense shaping terms (e.g., distance-to-target, orientation alignment) combined with sparse milestone bonuses (successful reach, grasp, transport, and place events)

### 3.2 Algorithms

Two actor-critic RL algorithms were implemented, trained, and benchmarked against one another under identical environment and reward conditions:

- **Proximal Policy Optimization (PPO)** — an on-policy algorithm valued for training stability
- **Soft Actor-Critic (SAC)** — an off-policy, maximum-entropy algorithm valued for sample efficiency

### 3.3 Results

| Metric                        | SAC        | PPO        |
|--------------------------------|------------|------------|
| Sample efficiency               | Higher     | Lower      |
| Final reward                    | Higher     | Lower      |
| Success rate                    | Higher     | Lower      |
| Path quality                    | Better     | Lower      |
| Early-exploration stability     | Lower      | Higher     |

SAC outperformed PPO on final task performance, while PPO showed more stable behavior during the early stages of training.

---

## 4. Perception Pipeline

A **YOLO-based perception pipeline** was designed for object detection and target localization, supporting two implementation paths:

- **RGB-D-based localization** — full 3D perception, providing direct depth information for accurate target pose estimation
- **Monocular localization (Raspberry Pi Camera Module 3)** — a low-cost alternative relying on monocular vision, intended as an accessible implementation option

---

## 5. Software Architecture

Two independent, parallel implementation tracks were developed:

### 5.1 ROS 2 Humble Workspace — [`ros2_mobile_manipulator_ws/`](./ros2_mobile_manipulator_ws/README.md)

A simulation-first, modular ROS 2 workspace built around dedicated packages:

- `mobile_manipulator_msgs` — shared custom interfaces (messages, services, actions)
- `simulation_pkg` — robot description, Gazebo world, RViz setup
- `perception_pkg` — vision pipeline and grasp target estimation
- `navigation_pkg` — localization, mapping, and navigation orchestration
- `manipulation_pkg` — kinematics and trajectory generation for arm control
- `task_manager_pkg` — task-level finite-state coordination and mission logic
- `hardware_interface_pkg` — hardware bridge for physical integration

### 5.2 Standalone Python Simulation Stack — [`mobile_manipulator_sim_python/`](./mobile_manipulator_sim_python/README.md)

A lightweight, ROS-independent Python pipeline (PyBullet-based physics, YOLO-based perception, FK/IK control, and a finite-state pick-and-place mission: `DETECT → MOVE_BASE → ALIGN_ARM → GRASP → TRANSPORT → PLACE → DONE`), used for rapid algorithm iteration alongside the ROS 2 integration track.

### 5.3 Reinforcement Learning Track — [`Matlab - reinforecment/`](./Matlab%20-%20reinforecment)

MATLAB-based RL environment, agent setup, and training scripts for the PPO/SAC comparison described in Section 3, including training-reward curves, evaluation histograms, and PPO-vs-SAC comparison plots.

---

## 6. Hardware Prototype

A physical prototype was built to validate the mechanical and control design, including:

- Arduino Mega-based motor and sensor control (`arduino mega/`)
- Custom electronics and wiring integration (`Fritzing/`)
- Chassis, arm, and camera-mount components fabricated per the SolidWorks CAD models (`CAD/`)

---

## 7. Tools and Technologies

- **Mechanical design:** SolidWorks
- **Simulation:** CoppeliaSim, Gazebo, PyBullet, custom Python-based RL training environment
- **Control / RL:** MATLAB, Python, PPO, SAC
- **Perception:** YOLO (Ultralytics)
- **Middleware:** ROS 2 Humble (simulation-first workspace)
- **Electronics:** Arduino Mega, Fritzing

---

## 8. Repository Structure

```
senior-project2/
├── CAD/                          # SolidWorks mechanical design and component models
├── Matlab - reinforecment/        # PPO/SAC RL environment, training, and evaluation
├── arduino mega/                   # Motor control and sensor firmware
├── copellia sim/                    # CoppeliaSim scene and scripts
├── mobile_manipulator_sim_python/    # Standalone Python simulation stack
├── ros2_mobile_manipulator_ws/        # ROS 2 Humble simulation-first workspace
├── Fritzing/                            # Electronics and wiring design
├── photo/                                # Prototype and simulation imagery
├── videos/                                # Demonstration and simulation recordings
├── poster/                                 # Graduation project poster
├── presentation/                           # Graduation project presentation
├── report/                                  # Full graduation project report
└── README.md
```

---

## 9. Relationship to Other Repository Projects

This project is the culmination of the academic progression documented in this repository, building on the kinematic and control foundations of the [semester project](../semester-project/README.md) and the custom mechanical/control design experience gained in the [senior project](../senior-project/README.md).

---

## 10. Authors

**Zain Alabidin Shbani**
Bachelor's Degree in Robotics and Intelligent Systems Engineering

**Beilassan Hdewa**
Bachelor's Degree in Robotics and Intelligent Systems Engineering

**Lana Alwazzeh**
Bachelor's Degree in Robotics and Intelligent Systems Engineering
