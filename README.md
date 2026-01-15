---

# SCARA 4-DOF Robot: Kinematics & Dynamics Simulation

This repository contains MATLAB simulations for a 4-DOF SCARA robot performing a pick-and-place task. It explores two different approaches to robotics: **Static Inverse Kinematics Optimization** and **Dynamic Trajectory Planning**.

## 🚀 Overview

The project simulates a 4-degree-of-freedom SCARA robot (3 Revolute joints, 1 Prismatic joint). The robot is programmed to move from a source "Pick" location to a target "Place" location using a safe clearance height ().

## 🛠️ Prerequisites

To run these simulations, you need:

* **MATLAB** (R2021a or later recommended).
* **Robotics Toolbox for MATLAB** by Peter Corke.
* **Optimization Toolbox** (for `fmincon` support).

## 📂 File Structure

Following standard naming conventions for MATLAB and GitHub:

* **`scara_static_ik_optimization.m`**: Focuses on precise Cartesian path following using static gravity compensation.
* **`scara_dynamic_trajectory_simulation.m`**: Focuses on smooth joint-space movement and full dynamic torque analysis using Newton-Euler equations.
* **`.gitignore`**: To prevent MATLAB temporary files from cluttering the repository.

## 📊 Technical Comparison

| Feature | Static Optimization Script | Dynamic Simulation Script |
| --- | --- | --- |
| **Path Style** | Straight-line (Cartesian Interpolation) | Curved/Smooth (Joint Interpolation) |
| **IK Solver** | Solves for every step (High CPU) | Solves for waypoints only (Efficient) |
| **Dynamics** | Gravity load only (`gravload`) | Full Inertia + Accel + Gravity (`rne`) |
| **Trajectory** | Linear steps | Quintic Polynomial (`jtraj`) |

## 🕹️ How to Run

1. Clone the repository:
```bash
git clone https://github.com/SatyaKrishna2811/4-DOF-SCARA-robot-for-Automated-biomedical-assays.git

```


2. Open MATLAB and navigate to the project folder.
3. Ensure the **Robotics Toolbox** is added to your path.
4. Run either of the `.m` scripts to view the 3D visualization and torque/force logs.

---

## 👥 Contributors

This project was developed as part of the **Mathematics for Intelligent Systems 3(MIS III)** course at **Amrita Vishwa Vidyapeetham**.

* **Vepuri Satya Krishna** (DL.AI.U4AID24140)
* **Gowripriya R** (DL.AI.U4AID24113)
* **Yaalini R** (DL.AI.U4AID24043)

---

### 📜 License
This project is for educational purposes. Feel free to use logic for your own learning!
