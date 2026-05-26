# Autonomous Mobile Manipulator for Pick-and-Place Tasks

Simulation-first ROS 2 Humble workspace for a modular autonomous mobile manipulator.

## Workspace layout

- `src/mobile_manipulator_msgs` – shared custom messages, services, and action interfaces
- `src/simulation_pkg` – robot description, Gazebo world, RViz support, simulation launch
- `src/perception_pkg` – camera, YOLO-style detector, grasp pose estimation
- `src/navigation_pkg` – localization, map publishing, obstacle avoidance, navigation bridge
- `src/manipulation_pkg` – FK/IK, trajectory generation, arm controller
- `src/task_manager_pkg` – finite-state task executor and pick-place action server
- `src/hardware_interface_pkg` – future hardware bridge stub

## File-by-file development order

1. `mobile_manipulator_msgs`
2. `simulation_pkg/urdf/mobile_manipulator.urdf.xacro`
3. `simulation_pkg/worlds/pick_and_place.world`
4. `simulation_pkg/launch/simulation.launch.py`
5. `perception_pkg` nodes and launch
6. `navigation_pkg` nodes and launch
7. `manipulation_pkg` nodes and launch
8. `task_manager_pkg` FSM and launch
9. `hardware_interface_pkg` bridge and launch
10. integration docs and zip packaging

## Build

```bash
cd /home/runner/work/Mobile-Manipulator-Simulation-Academic-Project-Journey/Mobile-Manipulator-Simulation-Academic-Project-Journey/ros2_mobile_manipulator_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch task_manager_pkg autonomous_pick_place.launch.py
```

## Validation

```bash
python3 -m compileall src
```

## Subsystem connections

- Camera publishes `/camera/image_raw` and `/camera/camera_info`.
- Perception publishes `/detections` and `/grasp_target_pose`.
- Navigation publishes `/map`, `/robot_pose`, and accepts `/navigate_to_pose`.
- Manipulation solves `/solve_ik` and `/solve_fk`, then publishes arm trajectories.
- Task manager coordinates perception, navigation, manipulation, and recovery behavior.
- Hardware bridge mirrors simulated commands for later embedded integration.

## Thesis documentation suggestions

- System requirements and assumptions
- ROS 2 software architecture
- Perception pipeline and evaluation
- Navigation and localization strategy
- Arm kinematics and trajectory generation
- FSM task orchestration and recovery logic
- Simulation validation results and limitations
- Hardware migration plan
