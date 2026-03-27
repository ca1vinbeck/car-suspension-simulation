# Car Suspension Simulation

This project simulates a **single-degree-of-freedom car suspension system** and explores the tradeoff between **comfort** and **handling**. It demonstrates how varying suspension parameters (spring stiffness `k` and damping coefficient `c`) affects system performance, and highlights optimal designs using **Pareto frontier analysis**.

---

## Project Overview

- **Objective:** Analyze how different suspension designs impact ride comfort, handling, and stability.  
- **Metrics computed:**  
  - **Comfort:** Maximum acceleration experienced by the car body  
  - **Handling:** Maximum displacement relative to the road  
  - **Stability:** Settling behavior (velocity over time)  
- **Methods:**  
  - Simulates system dynamics using **Runge-Kutta 4th order (RK4) numerical integration**  
  - Explores a range of spring and damping values  
  - Identifies **Pareto-optimal designs** balancing comfort and handling  

---

## Features

- Time-domain plots for **displacement, velocity, and acceleration**  
- Tradeoff scatter plot showing **comfort vs handling**  
- Highlights:
  - Best comfort (minimal acceleration)  
  - Best handling (minimal displacement)  
  - Balanced Pareto-optimal design  
- Fully commented Python code for easy modification and experimentation  

---

## How to Run

1. Clone the repository:
```bash
git clone https://github.com/YOUR_USERNAME/car-suspension-simulation.git
