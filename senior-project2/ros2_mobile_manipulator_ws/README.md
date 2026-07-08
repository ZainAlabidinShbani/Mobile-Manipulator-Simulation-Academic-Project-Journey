# Senior Project 2 - ROS 2 Autonomous Mobile Manipulator Workspace

This workspace contains the ROS 2 Humble integration track of Senior Project 2, my graduation project for the Bachelor's degree in Robotics and Intelligent Systems Engineering.

The architecture is simulation-first and modular, designed for autonomous pick-and-place workflows with a clear path toward hardware deployment.

## Prerequisites

- Ubuntu environment with ROS 2 Humble installed
- colcon build tools
- Gazebo and RViz for simulation and visualization
- Python dependencies required by individual ROS packages

## Workspace structure

- `src/mobile_manipulator_msgs` - Shared custom interfaces (messages, services, actions).
- `src/simulation_pkg` - Robot description, Gazebo world, RViz setup, simulation launch.
- `src/perception_pkg` - Vision pipeline and grasp target estimation.
- `src/navigation_pkg` - Localization, mapping, and navigation orchestration.
- `src/manipulation_pkg` - Kinematics and trajectory generation for arm control.
- `src/task_manager_pkg` - Task-level finite-state coordination and mission logic.
- `src/hardware_interface_pkg` - Hardware bridge stubs for future physical integration.

## Typical run workflow

1. Build and source the workspace.
2. Launch the autonomous pick-and-place system.
3. Monitor topics, transforms, and task transitions.
4. Validate package-level behavior and integration timing.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch task_manager_pkg autonomous_pick_place.launch.py
```

## Validation

```bash
python3 -m compileall src
```

## Suggested checks

- verify topic publication rates and message consistency
- verify IK/FK service responses under representative targets
- verify task manager transitions and recovery paths
- verify simulation behavior before hardware migration

## System communication summary

- Camera nodes provide image and camera info streams.
- Perception nodes publish object detections and grasp targets.
- Navigation modules process map and pose information and execute motion goals.
- Manipulation modules solve FK/IK and generate arm trajectories.
- Task manager coordinates all subsystems and handles recovery behavior.

## Why this workspace matters

This ROS 2 stack is the integration backbone of Senior Project 2. It was structured to support thesis-grade documentation, repeatable experiments, and migration from pure simulation to real robotic hardware.
