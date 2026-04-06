# Mobile Manipulator Simulation Projects

This repository contains **two robotics projects** — a semester project and a senior capstone project — both focused on mobile manipulator simulation. Each project lives in its own subfolder with its own README, code, and documentation.

---

## 📁 Projects

### 1. [Semester Project — 5-DOF KUKA youBot](./semester-project/README.md)

> 📂 `semester-project/`

A complete simulation framework for the **KUKA youBot mobile manipulator**, integrating its **omnidirectional mecanum base** and **5-DOF robotic arm**. Combines **ROS Noetic**, **MATLAB**, and **CoppeliaSim** for modeling, control, and visualization.

**Highlights:**
- 5-DOF robotic arm + mecanum-wheeled omnidirectional base
- Observer-based state feedback control
- Full ROS Noetic integration (URDF/Xacro, RViz, TF)
- CoppeliaSim 3D visualization
- Trajectory planning and manipulability analysis

---

### 2. [Senior Project — 3-DOF Custom Mobile Manipulator](./senior-project/README.md)

> 📂 `senior-project/`

A custom-designed **3-DOF robotic arm** on a mobile base, developed as a senior capstone project. Includes original CAD files and control code built on the foundations of the semester project.

**Highlights:**
- Custom-designed 3-DOF arm (CAD + code)
- Mobile base integration
- MATLAB-based control and simulation

---

## 🔍 Quick Comparison

| Feature | Semester Project | Senior Project |
|---|---|---|
| Robot | KUKA youBot | Custom Design |
| Arm DOF | 5 | 3 |
| Base | Mecanum-wheel (omnidirectional) | Mobile base |
| Tools | ROS + MATLAB + CoppeliaSim | MATLAB + CoppeliaSim |
| CAD | CATIA V5 → URDF | Custom CAD files |

---

## 🚀 Getting Started

Navigate to the project subfolder of your choice and follow its `README.md`:

```bash
# Semester project (KUKA youBot 5-DOF)
cd semester-project/

# Senior project (Custom 3-DOF arm)
cd senior-project/
```

---

## 👤 Author

**Zain Alabidin Shbani**  
Robotics & Mechatronics Engineering

---

## 📄 License

See [`semester-project/LICENSE`](./semester-project/LICENSE).
