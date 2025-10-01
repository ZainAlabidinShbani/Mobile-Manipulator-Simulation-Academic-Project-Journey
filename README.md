# KUKA youBot Mobile Manipulator Simulation Framework

🚀 A complete simulation framework for the **KUKA youBot mobile manipulator**, featuring an omnidirectional mecanum base, 5-DOF robotic arm, observer-based state feedback control, and multi-environment integration using ROS Noetic, MATLAB, and CoppeliaSim. Developed as part of our Bachelor's studies in **Robotics and Intelligent Systems Engineering** at **Manara University**.

---

## 📌 Project Overview

This project provides a full simulation environment for a mobile manipulator system capable of precise **pick-and-place operations** in static environments. The framework integrates **modeling, control, and visualization** across multiple platforms to ensure realistic performance and scalability for industrial and research applications.

---

## 📦 Project Structure

- `matlab/` : Core control, kinematics, and dynamics scripts. Entry point: `MobileManipulatorMain.m`.
- `Coppeliasim Scenes/` : `.ttt` scene files for 3D simulation in CoppeliaSim.
- `csv files/` : Example motion and control data for analysis and replay.
- `docs/` : Project documentation and final report (PDF).
- `media/` : Images and videos for documentation and demonstration.
<!-- - `ros/` : ROS Noetic robot description and integration files (URDF/Xacro, launch, nodes). -->

---

## ⚙️ Features

- 🟡 **Mecanum-wheeled base** for omnidirectional navigation
- 🟡 **5-DOF robotic arm** for accurate end-effector control
- 🟡 **Observer-Based State Feedback Control** for trajectory tracking
- 🟡 **Simulation & Visualization** in:
  - [CoppeliaSim](https://www.coppeliarobotics.com/)
  - [MATLAB](https://www.mathworks.com/products/matlab.html)
  - [ROS Noetic (RViz)](http://wiki.ros.org/noetic)
- 🟡 **Accurate CAD modeling** in CATIA V5 → translated into URDF/Xacro for ROS

---

## 📷 Media

All images are stored in `media/`. Below is a description and usage of each:

| Image                                                                                                 | Description                                             |
| ----------------------------------------------------------------------------------------------------- | ------------------------------------------------------- |
| `Denavit–Hartenberg Coordinate Frame Assignment for the KUKA youBot Arm.png`                          | DH frame assignment for all robotic arm joints.         |
| `Individual Wheel Angles.png`                                                                         | Steering angles of each mecanum wheel during motion.    |
| `Individual Wheel Velocities.png`                                                                     | Wheel velocities plotted over time.                     |
| `Manipulability Index over Time for the Combined System Jacobian.png`                                 | Manipulability analysis for full mobile manipulator.    |
| `Mobile Base Path.png`                                                                                | Path trajectory of the mobile base.                     |
| `Mobile Base State Trajectories.png`                                                                  | State variables of mobile base over time.               |
| `The entire image of the KUKA youBot shows both the mobile base and the 5-DOF arm with a gripper.png` | Full robot visualization combining arm and mobile base. |
| `Time Evolution of the End-Effector Twist in the body Frame.png`                                      | End-effector twist over time in body frame.             |
| `rosgraph.png`                                                                                        | ROS rqt_graph showing node and topic connections.       |
| `full robot assembled.png`                                                                            | Fully assembled robot visualization in ROS.             |

---

## 🚦 Usage Instructions

### 1. MATLAB Simulation

- Open MATLAB and navigate to the **matlab/** folder.
- Run **MobileManipulatorMain.m** to start the full simulation.
- Optional: Run additional scripts such as **ArmPolePlacementandObserver.m** to simulate only the arm control.

### 2. CoppeliaSim Visualization

- Open CoppeliaSim.
- Load the scene file **Coppeliasim_Scenes/Scene6_youBot_cube.ttt**.
- Ensure the robot parameters match MATLAB for consistent simulation.

### 3. Data Analysis

- CSV files in **csv_files/** contain motion and control data.
- MATLAB scripts can read and process these files for analysis or replay.

### 4. ROS Noetic Integration

#### 4.1. Navigate to the ROS package:

```bash
cd ros/kuka_youbot
```

````

#### 4.2. Build your ROS workspace (if not built yet):

```bash
cd ~/your_workspace_name
catkin_make
source devel/setup.bash
```

#### 4.3. Launch the full robot system:

```bash
roslaunch kuka_youbot full_robot.launch
```

#### Key Launch Components

- **robot_description**: Loads full URDF/Xacro model combining arm + mobile base.
- **robot_state_publisher**: Publishes all TF transforms.
- **joint_state_publisher_gui**: GUI for manual joint testing.
- **rviz**: Visualization using robot2.rviz config (shows robot, TF frames, markers, trajectory).
- **end_effector_pose.py**: Publishes 3D end-effector position using TF.
- **marker.py**: Visualizes gripper position in RViz.
- **path.py**: Updates end-effector trajectory in RViz.
- **rqt_graph**: Shows live node-topic communication.

## 👨‍👩‍👧‍👦 Team

- Beilassan Hdewa
- Lana Al Wazzeh
- Zain Alabidin Shbani

### 🧑‍🏫 Supervisors

- Dr. Mohamad Kheir Abdullah Mohamad
- Dr. Essa Alghannam

---

## 🔗 Hashtags

#Robotics #Simulation #ROS #CoppeliaSim #MATLAB #KUKAyouBot #MobileManipulation #Engineering #AI #ManaraUniversity #Teamwork #Research
````
