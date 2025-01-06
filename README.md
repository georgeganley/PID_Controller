# Acknowlegements
This project was completed as part of the AI for Robotics course at Georgia Tech. This repository does not include the code or solution to the project, but does illustrate the outcomes of a successful Proportional, Integral, and Derivitave (PID) controller implementation.

## Overview

Autonomous drones are widely used to inspect and maintain critical infrastructure (e.g., gas pipelines), where precise elevation and position control is essential. In this project, we focus on implementing and tuning two separate PID controllers:
1. **Thrust PID Controller** (for maintaining and adjusting the vertical position)
2. **Roll PID Controller** (for tilting the drone and controlling its horizontal position)

### Simplified Drone Model

- **Vertical movement (Thrust):**  
  - The drone has two rotors. When both rotors produce equal thrust, the drone maintains a level orientation.  
  - By increasing the thrust above the drone’s weight, the drone ascends. By reducing it, the drone descends.  
  - A PID controller computes the required thrust to reach and maintain the target elevation.

- **Horizontal movement (Roll):**  
  - To move horizontally, the drone must tilt (roll) left or right.  
  - For a dual-rotor drone, rolling is accomplished by varying the thrust between the left and right rotors.  
  - A positive roll angle tilts the drone to the left, resulting in leftward movement. A negative roll angle tilts the drone to the right, resulting in rightward movement.  
  - A separate PID controller computes the required roll angle to reach and maintain the target horizontal position.

### Problem Requirements

1. **Takeoff**  
   - The drone starts on the ground with both rotors off.  
   - It must lift off to the desired elevation within a specified time window.  

2. **Hover**  
   - Once the drone reaches the target elevation (and later the target horizontal position), it must maintain this position for a set duration.

3. **Horizontal Movement**  
   - Once airborne, the drone must tilt (roll) to move toward the target horizontal location.  
   - As the drone approaches the target, it needs to reduce the roll angle back to zero to hover in place.

## About PID Control

### What is PID?

A **PID** (Proportional-Integral-Derivative) controller is a common feedback control mechanism used in many industries to continuously calculate an error value as the difference between a desired setpoint and a measured process variable. The controller combines three terms:

- **Proportional (P)**: Directly proportional to the current error.  
- **Integral (I)**: Considers the accumulation of past error.  
- **Derivative (D)**: Predicts future error by observing the rate of change of the current error.

### PID Formula

For a given error \( e(t) \), the PID control output \( \alpha(t) \) is computed as follows:

$$\alpha(t) = \tau_p \cdot e(t) 
          + \tau_i \cdot \int e(t)\,dt 
          + \tau_d \cdot \frac{d}{dt} e(t)$$

where:
- $( \tau_p )$ is the proportional gain,
- $( \tau_i )$ is the integral gain,
- $( \tau_d )$ is the derivative gain,
- $( e(t) )$ is the error at time $( t )$.

In code form (often discretized for implementation):

$$\alpha = \tau_p \times \text{error} 
       + \tau_i \times \text{integral error}
       + \tau_d \times \text{derivative error}$$

### Relevance to the Drone Controller

1. **Thrust Control**:  
   - Error = (desired elevation) – (current elevation)  
   - Adjust thrust to ascend or descend as needed.  
   - The integral term helps correct any persistent offset.  
   - The derivative term anticipates sudden changes in vertical position or velocity.

2. **Roll Control**:  
   - Error = (desired horizontal position) – (current horizontal position)  
   - Adjust roll angle to tilt the drone left or right to move horizontally.
   - The integral term helps correct steady-state errors over time.  
   - The derivative term stabilizes and prevents oscillations in horizontal movement.

## Results

The visuals below illustrate the impact of a successfully implemented PID controller.

In the first example, below, the drone reaches and maintains a target height using the thrust PID controller, not needing to adjust for roll.

![Test_1](https://github.com/user-attachments/assets/9842342b-dc5a-400c-9208-c7678c0abc77)

In the second example, the drone reaches its target height, but is also impacted by an additional downward force. Simultaneously, the thrust PID must compensate for incremental loss in RPM over time, implementing an error in the drone for which the integral error must compensate.

![Test_2](https://github.com/user-attachments/assets/eb2387bf-ab31-4d21-a01a-0a5a61b38ad4)


