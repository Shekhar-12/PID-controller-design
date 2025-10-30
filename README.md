# PID Controller Design and Simulation using MATLAB

## 📖 Overview
This project demonstrates the design and simulation of a **PID (Proportional–Integral–Derivative) Controller** for a DC motor speed control system using **MATLAB**.  
The project compares the open-loop and closed-loop system performances, showing the effectiveness of PID tuning in improving response time, stability, and disturbance rejection.

---

## ⚙️ Project Details

### 🎯 Objective
To design and simulate a PID controller that improves the dynamic response of a DC motor by:
- Reducing rise time
- Minimizing overshoot
- Decreasing settling time
- Eliminating steady-state error

### 🧠 Theory
The **PID controller** combines three control actions:
\[
u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt}
\]
Where:
- \( K_p \) — Proportional gain  
- \( K_i \) — Integral gain  
- \( K_d \) — Derivative gain  
- \( e(t) \) — Error signal (difference between reference and actual output)

---

## 💻 Implementation

### Software Used
- **MATLAB** (no hardware required)
- **Control System Toolbox**

