# Kalman Filter for Object Tracking (Navigation)

## Project Overview
This project involves designing a Kalman filter for object tracking using MATLAB. The Kalman filter is a set of mathematical equations that provides an efficient computational (recursive) solution for the least-squares method, supporting estimations of past, present, and future states even when the precise nature of the modeled system is unknown. This project includes multiple examples to illustrate the implementation of Kalman filters for tracking in different dimensions.

## Table of Contents
1. [Introduction](#introduction)
2. [Theory](#theory)
3. [Software Requirements](#software-requirements)
4. [Project Structure](#project-structure)
5. [Implementation Details](#implementation-details)
    - [Example 1: Two-State Filter](#example-1-two-state-filter)
    - [Example 2: Three-State Filter](#example-2-three-state-filter)
    - [Example 3: Three-Dimensional Tracking](#example-3-three-dimensional-tracking)
6. [Usage](#usage)
7. [References](#references)

## Introduction
The Kalman filter, introduced by R.E. Kalman in 1960, addresses the problem of estimating the state of a discrete-time controlled process. It is powerful for its ability to provide estimations of past, present, and future states while handling unknown system dynamics. This project demonstrates the design and implementation of Kalman filters for object tracking in MATLAB.

## Theory
The Kalman filter uses a series of mathematical equations to estimate the state of a process. The filter operates in a recursive manner, utilizing time update (prediction) and measurement update (correction) equations. The time update equations project the current state and error covariance estimates forward in time, while the measurement update equations incorporate new measurements to correct these estimates.

### Kalman Filter Equations
1. **State Prediction:**
   ```math
    \hat{x}_{k|k-1} = A_k \hat{x}_{k-1|k-1} + B_k u_k
    
    P_{k|k-1} = A_k P_{k-1|k-1} A_k^T + Q_k
   ```

2. **Measurement Update:**
    ```math
    K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
    \hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H_k \hat{x}_{k|k-1})
    P_{k|k} = (I - K_k H_k) P_{k|k-1}
    ```

Where:
- \( \hat{x} \) is the state estimate.
- \( P \) is the estimate error covariance.
- \( A \) is the state transition matrix.
- \( B \) is the control input matrix.
- \( Q \) is the process noise covariance.
- \( K \) is the Kalman gain.
- \( H \) is the measurement matrix.
- \( R \) is the measurement noise covariance.
- \( z \) is the measurement vector.
- \( u \) is the control input vector.

## Software Requirements
- MATLAB

## Project Structure
The project is structured as follows:
- **Introduction:** Overview of the Kalman filter and its applications.
- **Theory:** Detailed explanation of the Kalman filter equations and concepts.
- **Examples:** Step-by-step implementation of the Kalman filter for different tracking scenarios.
    - [Example 1: Two-State Filter](#example-1-two-state-filter)
    - [Example 2: Three-State Filter](#example-2-three-state-filter)
    - [Example 3: Three-Dimensional Tracking](#example-3-three-dimensional-tracking)

## Implementation Details

### Example 1: Two-State Filter
#### Dynamic Model
The state of the vehicle is defined by position \( x \) and velocity \( v \). The equations are:
```math
x_{k+1} = x_k + v_k T + \frac{1}{2} a_k T^2
v_{k+1} = v_k + a_k T
```
Where a_k​ is the acceleration, a random variable.
####  Measurement Model
The position is measured with noise:
```math
z_k=H_k x_k+v_k
z_k = H_k x_k + v_k
Where H_k = [1, 0] and v_k​ is the measurement noise.
### Example 2: Three-State Filter
Similar to Example 1 but includes a third state, typically representing acceleration.
