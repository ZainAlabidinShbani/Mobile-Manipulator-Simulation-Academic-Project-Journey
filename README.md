# Mobile Manipulator Simulation: Academic Project Journey

This repository documents the progression of a mobile-manipulator project from an
initial KUKA youBot simulation, through a custom 3-DOF design, to an autonomous
4-DOF graduation-project prototype.

Each project is kept in its own directory with its own technical documentation.
Use this README as the project map, then follow the setup instructions in the
README for the project you want to run.

## Project map

| Stage | Project | Main focus | Primary technologies |
| --- | --- | --- | --- |
| 1 | [Semester project](./semester-project/README.md) | KUKA youBot modeling, kinematics, and observer-based control | MATLAB, ROS Noetic, CoppeliaSim |
| 2 | [Senior project](./senior-project/README.md) | Custom 3-DOF manipulator design and controller comparison | MATLAB, CAD |
| 3 | [Senior project 2](./senior-project2/README.md) | Autonomous pick-and-place with perception and reinforcement learning | ROS 2 Humble, Python, MATLAB, CoppeliaSim, Gazebo |

## How to use this repository

1. Choose a project from the table above.
2. Open that project's README for prerequisites and execution steps.
3. Install dependencies only for the selected project and toolchain.
4. Keep generated build products, virtual environments, and experiment outputs
   out of version control.

The projects are related chronologically, but they are not a single build
workspace. A project can be studied or executed independently once its
prerequisites are available.

## Progression

### Semester project

The first stage establishes the baseline mobile-manipulator workflow:

- 5-DOF KUKA youBot-style arm on a mecanum-wheeled base
- Forward and inverse kinematics and trajectory generation
- Observer-based state-feedback control
- MATLAB, ROS Noetic, and CoppeliaSim integration

### Senior project

The second stage moves from a reference platform to an original design:

- Custom 3-DOF arm mounted on a mobile base
- MATLAB kinematic and dynamic modeling
- PID, LQR, and Fuzzy-PID controller comparison
- CAD-based mechanical design and feasibility analysis

### Senior project 2

The graduation-project stage extends the system toward autonomous operation:

- 4-DOF parallelogram-linkage arm on a four-wheel skid-steer base
- YOLO-based object detection and target localization
- PPO and SAC reinforcement-learning experiments
- ROS 2 integration and a standalone Python simulation stack
- Hardware-oriented CAD and electronics work

See the [Senior Project 2 README](./senior-project2/README.md) for the
architecture and links to its ROS 2 and Python implementation tracks.

## Repository structure

```text
.
├── semester-project/   # KUKA youBot simulation and control
├── senior-project/     # Custom 3-DOF manipulator design
├── senior-project2/    # Autonomous mobile-manipulator graduation project
└── README.md           # This project map
```

## Related repository

The later ROS 2 warehouse-simulation work is maintained separately in
[mobile_manipulator_ws](https://github.com/ZainAlabidinShbani/mobile_manipulator_ws).

## Authors

- Zain Alabidin Shbani
- Beilassan Hdewa
- Lana Alwazzeh

## Notes

This is an academic and research portfolio repository. Results, interfaces,
and experiment configurations may change as the individual projects evolve.
Refer to each project's documentation and reports for the authoritative
technical details.
