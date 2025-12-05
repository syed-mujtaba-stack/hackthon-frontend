---
sidebar_position: 10
---

# Chapter 9: Control & Balance for Bipedal Robots

## Learning Objectives

- Understand bipedal locomotion challenges
- Master PID control for joint-level control
- Implement Model Predictive Control (MPC) for walking
- Learn Whole-Body Control (WBC) for multi-task coordination
- Implement Zero Moment Point (ZMP) based balance
- Deploy walking controllers in simulation

## The Challenge of Bipedal Locomotion

Humans make walking look effortless, but it's one of the hardest problems in robotics.

### Why is Bipedal Walking Hard?

| Challenge | Description |
|-----------|-------------|
| **Underactuation** | Fewer actuators than degrees of freedom (can't control everything) |
| **Instability** | Small support polygon (two feet) |
| **Hybrid Dynamics** | Continuous (swing phase) + discrete (foot contact) |
| **High DOF** | 12+ joints just for legs |
| **Energy Efficiency** | Batteries are limited |

### Human vs. Robot Walking

| Metric | Human | Typical Humanoid |
|--------|-------|------------------|
| **Energy Cost of Transport** | 0.2 | 0.8-2.0 |
| **Walking Speed** | 1.4 m/s | 0.3-0.8 m/s |
| **Stability** | Excellent | Fragile |
| **Adaptability** | High (stairs, slopes) | Limited |

## PID Control: The Foundation

**PID (Proportional-Integral-Derivative)** is the most common controller in robotics.

### PID Equation

```
u(t) = Kp * e(t) + Ki * ∫e(t)dt + Kd * de(t)/dt
```

Where:
- `e(t)` = error (desired - actual)
- `Kp` = proportional gain
- `Ki` = integral gain
- `Kd` = derivative gain

### Implementing PID in Python

```python
class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        
        self.integral = 0.0
        self.prev_error = 0.0
    
    def compute(self, setpoint, measurement):
        # Error
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral += error * self.dt
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        d_term = self.kd * derivative
        
        # Update
        self.prev_error = error
        
        # Control output
        output = p_term + i_term + d_term
        return output
```

### Tuning PID

**Ziegler-Nichols Method**:

1. Set `Ki = 0`, `Kd = 0`
2. Increase `Kp` until system oscillates
3. Record `Ku` (ultimate gain) and `Tu` (oscillation period)
4. Set:
   - `Kp = 0.6 * Ku`
   - `Ki = 2 * Kp / Tu`
   - `Kd = Kp * Tu / 8`

### PID for Joint Control

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class JointPIDController(Node):
    def __init__(self):
        super().__init__('joint_pid_controller')
        
        # PID controller for knee joint
        self.pid = PIDController(kp=50.0, ki=0.1, kd=5.0)
        
        # Subscribers and publishers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.cmd_pub = self.create_publisher(
            Float64, '/knee_effort_controller/command', 10)
        
        self.desired_angle = 0.5  # radians
    
    def joint_callback(self, msg):
        # Find knee joint
        try:
            idx = msg.name.index('knee')
            current_angle = msg.position[idx]
            
            # Compute control
            effort = self.pid.compute(self.desired_angle, current_angle)
            
            # Publish
            cmd = Float64()
            cmd.data = effort
            self.cmd_pub.publish(cmd)
            
        except ValueError:
            pass
```

## Zero Moment Point (ZMP)

**ZMP** is a stability criterion for bipedal walking.

### Definition

The ZMP is the point on the ground where the net moment from gravity and inertia is zero.

**Stability Condition**: ZMP must stay inside the **support polygon** (convex hull of foot contacts).

```
If ZMP is inside support polygon → Stable
If ZMP is outside → Robot will tip over
```

### Calculating ZMP

```python
import numpy as np

def calculate_zmp(com_position, com_acceleration, total_mass, g=9.81):
    """
    Calculate ZMP x-coordinate
    
    Args:
        com_position: [x, y, z] center of mass position
        com_acceleration: [ax, ay, az] COM acceleration
        total_mass: robot mass (kg)
        g: gravity (m/s^2)
    
    Returns:
        zmp_x: ZMP x-coordinate
    """
    zmp_x = com_position[0] - (com_position[2] / (com_acceleration[2] + g)) * com_acceleration[0]
    return zmp_x
```

### ZMP-Based Walking

**Linear Inverted Pendulum Model (LIPM)**:

```python
class LIPMWalking:
    def __init__(self, com_height=0.8, step_time=0.8):
        self.h = com_height  # COM height
        self.T = step_time   # Step duration
        self.g = 9.81
        
        # Natural frequency
        self.omega = np.sqrt(self.g / self.h)
    
    def compute_com_trajectory(self, zmp_ref):
        """
        Compute COM trajectory to track ZMP reference
        """
        # Simplified: COM position to achieve desired ZMP
        com_x = zmp_ref + self.h / self.g * 0  # Simplified (full version uses differential equations)
        return com_x
```

## Model Predictive Control (MPC)

**MPC** optimizes future control actions over a time horizon.

### MPC for Walking

```python
import cvxpy as cp

class MPCWalkingController:
    def __init__(self, horizon=10, dt=0.1):
        self.N = horizon  # Prediction horizon
        self.dt = dt
    
    def solve(self, current_state, zmp_reference):
        """
        Solve MPC optimization problem
        
        Args:
            current_state: [com_x, com_vx]
            zmp_reference: desired ZMP trajectory (N steps)
        
        Returns:
            optimal_control: COM acceleration commands
        """
        # State: [position, velocity]
        x = cp.Variable((2, self.N + 1))
        u = cp.Variable(self.N)  # Control (COM acceleration)
        
        # Dynamics: x[k+1] = A*x[k] + B*u[k]
        A = np.array([[1, self.dt], [0, 1]])
        B = np.array([[0.5 * self.dt**2], [self.dt]])
        
        # Cost function
        cost = 0
        constraints = [x[:, 0] == current_state]
        
        for k in range(self.N):
            # Track ZMP reference
            zmp_k = x[0, k] - (0.8 / 9.81) * u[k]  # Simplified ZMP equation
            cost += cp.square(zmp_k - zmp_reference[k])
            
            # Dynamics constraint
            constraints += [x[:, k+1] == A @ x[:, k] + B.flatten() * u[k]]
            
            # Control limits
            constraints += [cp.abs(u[k]) <= 2.0]  # Max acceleration
        
        # Solve
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()
        
        return u.value
```

## Whole-Body Control (WBC)

**WBC** coordinates all joints to achieve multiple tasks simultaneously.

### Task Hierarchy

1. **High Priority**: Balance (ZMP, COM)
2. **Medium Priority**: Foot placement
3. **Low Priority**: Arm swing, head orientation

### Quadratic Programming Formulation

```python
import qpsolvers

class WholeBodyController:
    def __init__(self, num_joints=12):
        self.n = num_joints
    
    def solve(self, tasks, constraints):
        """
        Solve WBC as QP:
        
        minimize: ||J*q_ddot - task_acc||^2
        subject to: dynamics and joint limits
        
        Args:
            tasks: list of (Jacobian, desired_acceleration, weight)
            constraints: joint limits, contact forces
        
        Returns:
            q_ddot: joint accelerations
        """
        # Build QP matrices
        H = np.eye(self.n)  # Quadratic cost
        f = np.zeros(self.n)  # Linear cost
        
        for J, task_acc, weight in tasks:
            H += weight * (J.T @ J)
            f -= weight * (J.T @ task_acc)
        
        # Solve QP
        q_ddot = qpsolvers.solve_qp(H, f, G=None, h=None)
        return q_ddot
```

## Lab Exercise: Implementing a Walking Controller

### Objective
Implement a simple ZMP-based walking controller for a simulated humanoid.

### Step 1: Define Gait Pattern

```python
class GaitGenerator:
    def __init__(self):
        self.step_length = 0.2  # meters
        self.step_height = 0.05  # meters
        self.step_time = 0.8    # seconds
    
    def generate_footsteps(self, num_steps=4):
        footsteps = []
        for i in range(num_steps):
            x = i * self.step_length
            y = 0.1 if i % 2 == 0 else -0.1  # Alternate feet
            footsteps.append((x, y))
        return footsteps
```

### Step 2: ZMP Trajectory

```python
def generate_zmp_trajectory(footsteps, dt=0.01):
    zmp_traj = []
    for i in range(len(footsteps) - 1):
        # ZMP stays at support foot during swing
        start = footsteps[i]
        end = footsteps[i + 1]
        
        # Linear interpolation
        steps = int(0.8 / dt)  # 0.8 seconds per step
        for t in range(steps):
            alpha = t / steps
            zmp_x = start[0] + alpha * (end[0] - start[0])
            zmp_traj.append(zmp_x)
    
    return zmp_traj
```

### Step 3: MPC Controller

```python
controller = MPCWalkingController(horizon=10, dt=0.1)

current_state = np.array([0.0, 0.0])  # [position, velocity]
zmp_ref = generate_zmp_trajectory(footsteps)

for k in range(len(zmp_ref) - 10):
    # Solve MPC
    u_opt = controller.solve(current_state, zmp_ref[k:k+10])
    
    # Apply first control action
    com_acc = u_opt[0]
    
    # Update state (simplified dynamics)
    current_state[1] += com_acc * 0.1  # velocity
    current_state[0] += current_state[1] * 0.1  # position
    
    print(f"Step {k}: COM position = {current_state[0]:.3f}, ZMP ref = {zmp_ref[k]:.3f}")
```

### Step 4: Simulate in Gazebo

```bash
ros2 launch humanoid_walking walking_controller.launch.py
```

## Quiz

1. **What does PID stand for?**
   - A) Proportional-Integral-Differential
   - B) Proportional-Integral-Derivative
   - C) Position-Inertia-Damping
   - D) Power-Input-Device
   
   **Answer: B**

2. **What is the ZMP stability condition?**
   - A) ZMP must be at the center of mass
   - B) ZMP must be inside the support polygon
   - C) ZMP must be zero
   - D) ZMP must be at the foot
   
   **Answer: B**

3. **What is the advantage of MPC over PID?**
   - A) Simpler to implement
   - B) Optimizes over future time horizon
   - C) Requires less computation
   - D) Works without sensors
   
   **Answer: B**

4. **What is Whole-Body Control used for?**
   - A) Controlling a single joint
   - B) Coordinating all joints for multiple tasks
   - C) Only for arm control
   - D) Measuring robot weight
   
   **Answer: B**

## Summary

In this chapter, we mastered control theory for bipedal humanoid robots. We learned PID control for joint-level control, ZMP for balance analysis, MPC for optimal walking, and Whole-Body Control for multi-task coordination. We implemented a simple walking controller using ZMP and MPC. Control is the bridge between planning and execution, enabling humanoids to move gracefully and safely.

**Next Chapter**: We'll integrate conversational AI with humanoid robots using speech recognition and language models.
