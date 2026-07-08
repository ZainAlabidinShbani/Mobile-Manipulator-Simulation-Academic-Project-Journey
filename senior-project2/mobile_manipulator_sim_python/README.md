# Senior Project 2 - Autonomous Mobile Manipulator (Python Stack)

This directory contains the standalone Python simulation branch of Senior Project 2, developed as part of my graduation project in Robotics and Intelligent Systems Engineering.

It delivers a complete autonomous mobile manipulator pipeline in Python, including simulation, perception, planning, control, task orchestration, GUI monitoring, and data logging.

## Scope

- No ROS dependency in this branch
- No MATLAB dependency in this branch
- Full execution from `main.py`

## Intended use

This branch is designed for rapid experimentation and algorithm iteration, especially when a lightweight, ROS-free execution path is preferred for testing autonomy logic and controller behavior.

## Core capabilities

- Physics simulation with PyBullet and safe fallback behavior when PyBullet is unavailable
- Vision pipeline with YOLO-based detection and fallback detector support
- Mobile base and arm control with FK/IK and numerical inverse kinematics
- Pick-and-place planning with trajectory generation in joint and Cartesian spaces
- Finite-state mission execution:
  `DETECT -> MOVE_BASE -> ALIGN_ARM -> GRASP -> TRANSPORT -> PLACE -> DONE`
- Real-time GUI dashboards and plotting
- CSV logging for offline analysis and report generation

## Project layout

```
mobile_manipulator_sim_python/
├── main.py
├── config.py
├── simulation/
├── control/
├── perception/
├── planning/
├── tasks/
├── logging_utils/
├── gui/
├── data/
└── requirements.txt
```

## Quick start

```bash
pip install -r requirements.txt
python main.py
python main.py --advanced
python main.py --headless
```

## Main dependencies

- `numpy`
- `scipy`
- `matplotlib`
- `pandas`
- `PyQt5`
- `pybullet` (optional but recommended)
- `ultralytics` (optional for YOLO-based detection)

## GUI modes

- Basic mode (`python main.py`): control buttons plus live technical plots.
- Advanced mode (`python main.py --advanced`): animated robot scene, motion indicators, end-effector metrics, and richer debugging instrumentation.
- Headless mode (`python main.py --headless`): useful for batch runs and fast testing.

## Configuration notes

Arm degrees of freedom are configurable from `config.py` through:

- `ARM_NUM_JOINTS`
- `ARM_LINK_LENGTHS`
- `ARM_JOINT_LOWER`
- `ARM_JOINT_UPPER`
- `ARM_HOME_CONFIG`

The kinematics pipeline is built to adapt automatically when these parameters are changed consistently.

## Expected outputs

- real-time task progression in GUI mode
- robot state and trajectory plots
- CSV logs in `data/` for post-run analysis
- reproducible state-machine execution traces for reporting

## Practical use in the graduation project

This Python stack was used to iterate quickly on autonomous behavior logic, evaluate perception-control interaction, and validate task-level performance before and alongside ROS-based integration.
