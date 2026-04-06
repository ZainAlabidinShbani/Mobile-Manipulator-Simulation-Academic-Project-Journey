# Senior Project — 3-DOF Mobile Manipulator

This senior capstone project develops a **custom 3-DOF robotic arm** mounted on a mobile base. It extends the concepts from the [semester project (5-DOF KUKA youBot)](../semester-project/README.md) and focuses on original mechanical design plus MATLAB-based modeling and control.

---

## 📌 Project Overview

- **System**: Mobile base + custom 3-DOF arm.
- **Focus**: Kinematics, dynamics, and controller design in MATLAB.
- **Design assets**: Full CAD models for the arm and base.

---

## 📦 Project Structure

| Folder    | Description                                           |
| --------- | ----------------------------------------------------- |
| `matlab/` | Control, kinematics, dynamics, and simulation scripts |
| `cad/`    | CAD design files for the custom arm and base          |
| `docs/`   | Project documentation and report material             |
| `media/`  | Figures and media used in documentation               |

---

## 🚦 Usage Instructions

### MATLAB Simulation

1. Open MATLAB and set the working directory to `senior-project/matlab/`.
2. Run the main script for the simulation and analysis.

### CAD Files

- Open the files in `cad/` with your preferred CAD software (e.g., SolidWorks).

---

## ⚙️ Key Differences from Semester Project

| Feature    | Semester Project (5-DOF KUKA) | Senior Project (3-DOF Custom) |
| ---------- | ----------------------------- | ----------------------------- |
| Robot      | KUKA youBot                   | Custom-designed               |
| DOF        | 5                             | 3                             |
| Base       | Mecanum-wheel omnidirectional | Mobile base                   |
| CAD source | CATIA V5 → URDF               | Custom CAD files              |
| Simulator  | CoppeliaSim + ROS + MATLAB    | MATLAB + CoppeliaSim          |

---

## 📷 Media

See the `media/` folder for figures and documentation visuals.
