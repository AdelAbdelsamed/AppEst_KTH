# AppEst_KTH

# Applied Estimation (AppEst) - EL2320 (Autumn 2024) Coursework Repository

This repository contains the lab assignments and final project for the EL2320 course at KTH during the autumn term of 2024.

## Course Overview

The coursework consists of three milestones:

---

### Lab Assignments

#### 1. **Lab 1 - Extended Kalman Filter (EKF) Localization**  
This lab focuses on implementing and analyzing Kalman Filter (KF) and Extended Kalman Filter (EKF) algorithms for state estimation and robot localization. It involves preparatory theoretical questions and practical MATLAB exercises to deepen understanding.

**Key Objectives:**
- **Warm-Up Exercise:** Exploring the behavior of the standard Kalman Filter in a car-tracking scenario.  
- **Main Problem:** Developing an EKF-based localization system for a robot, addressing nonlinearity in the dynamics and measurement model. Maximum Likelihood is used for data association, and outlier detection is based on the Mahalanobis distance.

---

#### 2. **Lab 2 - Monte Carlo Localization (MCL) and Particle Filters**  
This lab builds on the concepts from Lab 1, focusing on Particle Filters (PF) and Monte Carlo Localization (MCL) for robot tracking and global localization. Through preparatory questions, MATLAB exercises, and simulations, the lab explores probabilistic state estimation in depth.

**Key Objectives:**
- **Warm-Up Exercise:** Gaining insights into the Sampling-Importance Resampling (SIR) particle filter through simple target tracking problems.  
- **Main Problem:** Implementing a Monte Carlo Localization algorithm for both tracking and global localization scenarios while addressing challenges resulting from symmetry.

---

### Final Project

#### **Comparative Analysis of State Estimation Techniques for a DC Motor**  
This project evaluates state estimation techniques for DC motors, focusing on the challenges posed by static friction nonlinearity. The study compares five filters: Kalman Filter (KF), Extended Kalman Filter (EKF), Unscented Kalman Filter (UKF), Particle Filter (PF), and Marginalized Particle Filter (MPF).

**Key Contributions:**
- Incorporation of static friction nonlinearity into the estimation models.  
- Dual validation framework using both simulation and experimental data.  
- Comprehensive analysis of accuracy, robustness, and computational efficiency across the estimators.  

**Highlights:**  
Experimental results demonstrate the superiority of nonlinear estimators, particularly the UKF and MPF, in capturing rotor dynamics in low-speed regions where static friction dominates. These findings provide valuable insights for applications requiring precise motion control with minimal sensor usage.

---
