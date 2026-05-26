# Architecture Overview

1. `simulation_pkg` provides the robot model, Gazebo world, and visualization bridge.
2. `perception_pkg` synthesizes camera data, detects targets, and estimates grasp poses.
3. `navigation_pkg` localizes the robot, publishes a map, filters motion commands, and drives goals.
4. `manipulation_pkg` solves arm kinematics, generates trajectories, and simulates execution.
5. `task_manager_pkg` sequences the full pick-and-place workflow with retries and recovery.
6. `hardware_interface_pkg` mirrors the ROS interfaces for future hardware deployment.

The system is designed so each package can be launched independently during integration and then combined under the task manager launch file.
