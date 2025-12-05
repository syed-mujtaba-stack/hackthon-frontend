---
sidebar_position: 4
---

# Chapter 3: URDF and Xacro for Humanoid Robots

## Learning Objectives

- Master URDF syntax for robot description
- Understand links, joints, and their properties
- Use Xacro for modular, maintainable robot models
- Calculate inertial properties for realistic simulation
- Visualize and debug URDF models in RViz
- Build a complete humanoid leg model

## The Unified Robot Description Format (URDF)

URDF is the standard XML format in ROS for describing:
- **Robot structure** (links and joints)
- **Visual appearance** (meshes, colors)
- **Collision geometry** (simplified shapes for physics)
- **Physical properties** (mass, inertia)
- **Sensor and actuator placement**

### Why URDF Matters for Humanoids

Humanoid robots have:
- **High DOF**: 25-40 joints (vs. 6 for industrial arms)
- **Complex kinematics**: Closed-loop chains, parallel mechanisms
- **Strict mass budgets**: Every gram affects balance

A well-crafted URDF enables:
- Accurate simulation in Gazebo/Isaac Sim
- Correct inverse kinematics (IK) solutions
- Realistic physics (inertia, friction, damping)

## URDF Structure Deep Dive

### Links

A **link** represents a rigid body. It has three components:

#### 1. Visual (Appearance)
```xml
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://my_robot/meshes/thigh.stl" scale="1 1 1"/>
    <!-- OR -->
    <cylinder radius="0.05" length="0.4"/>
  </geometry>
  <material name="carbon_fiber">
    <color rgba="0.2 0.2 0.2 1.0"/>
  </material>
</visual>
```

#### 2. Collision (Physics)
```xml
<collision>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <!-- Simplified geometry for faster collision detection -->
    <cylinder radius="0.06" length="0.4"/>
  </geometry>
</collision>
```

**Best Practice**: Use simplified shapes (boxes, cylinders, spheres) for collision geometry to improve simulation performance.

#### 3. Inertial (Dynamics)
```xml
<inertial>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>  <!-- Center of mass -->
  <mass value="2.5"/>
  <inertia ixx="0.0283" ixy="0.0" ixz="0.0"
           iyy="0.0283" iyz="0.0"
           izz="0.00125"/>
</inertial>
```

**Calculating Inertia**: For a cylinder of mass `m`, radius `r`, length `h`:
- `Ixx = Iyy = (1/12) * m * h² + (1/4) * m * r²`
- `Izz = (1/2) * m * r²`

### Joints

A **joint** connects two links and defines their relative motion.

#### Joint Types

| Type | Description | Use Case |
|------|-------------|----------|
| `revolute` | Rotation with limits | Knee, elbow |
| `continuous` | Unlimited rotation | Wheel axle |
| `prismatic` | Linear motion | Telescoping arm |
| `fixed` | No motion | Sensor mount |
| `floating` | 6-DOF (x, y, z, roll, pitch, yaw) | Free-floating base |
| `planar` | 2D motion in a plane | Rarely used |

#### Revolute Joint Example (Hip)

```xml
<joint name="left_hip_pitch" type="revolute">
  <parent link="pelvis"/>
  <child link="left_thigh"/>
  <origin xyz="0.0 0.1 -0.05" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation around Y-axis -->
  <limit lower="-1.57" upper="1.57" effort="150" velocity="3.0"/>
  <dynamics damping="0.7" friction="0.5"/>
</joint>
```

**Parameters**:
- `lower/upper`: Joint limits (radians)
- `effort`: Maximum torque (Nm)
- `velocity`: Maximum angular velocity (rad/s)
- `damping`: Viscous damping coefficient
- `friction`: Coulomb friction

## Xacro: Macros for URDF

**Xacro** (XML Macros) extends URDF with:
- **Variables**: Define constants (e.g., `thigh_length`)
- **Macros**: Reusable templates (e.g., `leg_macro`)
- **Math**: Calculate values (e.g., `${thigh_length / 2}`)
- **Includes**: Split large models into files

### Example: Xacro Variables

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid">
  
  <!-- Constants -->
  <xacro:property name="thigh_length" value="0.4"/>
  <xacro:property name="thigh_radius" value="0.05"/>
  <xacro:property name="thigh_mass" value="2.5"/>
  
  <!-- Calculated inertia -->
  <xacro:property name="thigh_ixx" value="${(1/12) * thigh_mass * thigh_length * thigh_length + (1/4) * thigh_mass * thigh_radius * thigh_radius}"/>
  
  <link name="left_thigh">
    <visual>
      <geometry>
        <cylinder radius="${thigh_radius}" length="${thigh_length}"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="${thigh_mass}"/>
      <inertia ixx="${thigh_ixx}" ixy="0" ixz="0"
               iyy="${thigh_ixx}" iyz="0"
               izz="${(1/2) * thigh_mass * thigh_radius * thigh_radius}"/>
    </inertial>
  </link>
  
</robot>
```

### Example: Xacro Macro (Reusable Leg)

```xml
<xacro:macro name="leg" params="prefix reflect">
  
  <!-- Thigh Link -->
  <link name="${prefix}_thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <inertia ixx="0.0283" ixy="0" ixz="0" iyy="0.0283" iyz="0" izz="0.00125"/>
    </inertial>
  </link>
  
  <!-- Hip Joint -->
  <joint name="${prefix}_hip_pitch" type="revolute">
    <parent link="pelvis"/>
    <child link="${prefix}_thigh"/>
    <origin xyz="0 ${reflect * 0.1} -0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="150" velocity="3.0"/>
  </joint>
  
  <!-- Shank Link -->
  <link name="${prefix}_shank">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1.8"/>
      <inertia ixx="0.024" ixy="0" ixz="0" iyy="0.024" iyz="0" izz="0.00072"/>
    </inertial>
  </link>
  
  <!-- Knee Joint -->
  <joint name="${prefix}_knee" type="revolute">
    <parent link="${prefix}_thigh"/>
    <child link="${prefix}_shank"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="3.0"/>
  </joint>
  
  <!-- Foot Link -->
  <link name="${prefix}_foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>
  </link>
  
  <!-- Ankle Joint -->
  <joint name="${prefix}_ankle" type="revolute">
    <parent link="${prefix}_shank"/>
    <child link="${prefix}_foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7" upper="0.7" effort="50" velocity="2.0"/>
  </joint>
  
</xacro:macro>

<!-- Instantiate both legs -->
<xacro:leg prefix="left" reflect="1"/>
<xacro:leg prefix="right" reflect="-1"/>
```

**Explanation**:
- `prefix`: "left" or "right"
- `reflect`: 1 or -1 to mirror the leg position

## Building a Complete Humanoid Model

### Step-by-Step: Humanoid Torso + Legs

Create `humanoid.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_humanoid">
  
  <!-- Include leg macro -->
  <xacro:include filename="$(find my_robot)/urdf/leg_macro.xacro"/>
  
  <!-- Base Link (required for ROS) -->
  <link name="base_link"/>
  
  <!-- Pelvis -->
  <link name="pelvis">
    <visual>
      <geometry>
        <box size="0.3 0.25 0.15"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="8.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.06" iyz="0" izz="0.08"/>
    </inertial>
  </link>
  
  <joint name="base_to_pelvis" type="fixed">
    <parent link="base_link"/>
    <child link="pelvis"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>
  </joint>
  
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <inertial>
      <mass value="12.0"/>
      <inertia ixx="0.3" ixy="0" ixz="0" iyy="0.35" iyz="0" izz="0.15"/>
    </inertial>
  </link>
  
  <joint name="waist" type="revolute">
    <parent link="pelvis"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>  <!-- Yaw rotation -->
    <limit lower="-0.5" upper="0.5" effort="80" velocity="1.5"/>
  </joint>
  
  <!-- Instantiate legs -->
  <xacro:leg prefix="left" reflect="1"/>
  <xacro:leg prefix="right" reflect="-1"/>
  
</robot>
```

### Converting Xacro to URDF

```bash
xacro humanoid.urdf.xacro > humanoid.urdf
```

## Visualizing in RViz

```bash
# Install joint state publisher GUI
sudo apt install ros-humble-joint-state-publisher-gui

# Launch visualization
ros2 launch urdf_tutorial display.launch.py model:=humanoid.urdf
```

**What you'll see**:
- 3D model of the robot
- Sliders to move each joint
- TF (transform) tree showing link relationships

## Common URDF Mistakes

### 1. Incorrect Inertia Tensors

**Problem**: Simulation explodes or robot vibrates uncontrollably.

**Solution**: Use realistic inertia values. Tools:
- [MeshLab](https://www.meshlab.net/) for mesh analysis
- [SolidWorks](https://www.solidworks.com/) exports URDF with correct inertia

### 2. Collision Geometry Too Complex

**Problem**: Simulation runs slowly (< 1x real-time).

**Solution**: Use primitive shapes (boxes, cylinders) for collision.

### 3. Missing `base_link`

**Problem**: ROS tools expect a `base_link` as the root.

**Solution**: Always include a `base_link`, even if it's just a fixed joint to the pelvis.

### 4. Joint Limits Too Tight

**Problem**: Robot can't perform natural motions.

**Solution**: Reference human joint ranges:
- Hip: ±120° (flexion/extension)
- Knee: 0° to 135° (extension to flexion)
- Ankle: ±45° (dorsiflexion/plantarflexion)

## Lab Exercise: Build a 3-DOF Leg

### Objective
Create a URDF for a humanoid leg with:
- Hip (1 DOF: pitch)
- Knee (1 DOF: pitch)
- Ankle (1 DOF: pitch)

### Step 1: Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake humanoid_leg_description
cd humanoid_leg_description
mkdir urdf meshes launch
```

### Step 2: Write URDF

Create `urdf/leg.urdf.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_leg">
  
  <link name="base_link"/>
  
  <!-- Thigh -->
  <link name="thigh">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="2.5"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.0283" ixy="0" ixz="0" iyy="0.0283" iyz="0" izz="0.00125"/>
    </inertial>
  </link>
  
  <joint name="hip" type="revolute">
    <parent link="base_link"/>
    <child link="thigh"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="2.09" effort="150" velocity="3.0"/>
  </joint>
  
  <!-- Shank -->
  <link name="shank">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="1.8"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <inertia ixx="0.024" ixy="0" ixz="0" iyy="0.024" iyz="0" izz="0.00072"/>
    </inertial>
  </link>
  
  <joint name="knee" type="revolute">
    <parent link="thigh"/>
    <child link="shank"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="3.0"/>
  </joint>
  
  <!-- Foot -->
  <link name="foot">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.05"/>
      </geometry>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>
  </link>
  
  <joint name="ankle" type="revolute">
    <parent link="shank"/>
    <child link="foot"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.7" upper="0.7" effort="50" velocity="2.0"/>
  </joint>
  
</robot>
```

### Step 3: Create Launch File

Create `launch/display.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_leg_description'),
        'urdf', 'leg.urdf.xacro')
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('humanoid_leg_description'), 'rviz', 'leg.rviz')]
        )
    ])
```

### Step 4: Build and Launch

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_leg_description
source install/setup.bash
ros2 launch humanoid_leg_description display.launch.py
```

### Tasks
1. Move the hip, knee, and ankle joints using the GUI
2. Verify joint limits are respected
3. Add a second leg using a Xacro macro
4. Calculate and verify inertia values

## Quiz

1. **What is the purpose of the `<collision>` element in URDF?**
   - A) Visual appearance
   - B) Simplified geometry for physics simulation
   - C) Mass properties
   - D) Joint limits
   
   **Answer: B**

2. **Which joint type allows unlimited rotation?**
   - A) `revolute`
   - B) `prismatic`
   - C) `continuous`
   - D) `fixed`
   
   **Answer: C**

3. **What does Xacro provide over plain URDF?**
   - A) Faster parsing
   - B) Variables, macros, and math
   - C) Better visualization
   - D) Automatic inertia calculation
   
   **Answer: B**

4. **Why use simplified collision geometry?**
   - A) Looks better
   - B) Easier to edit
   - C) Faster simulation
   - D) Required by ROS
   
   **Answer: C**

## Summary

In this chapter, we mastered URDF for describing robot structure, including links (visual, collision, inertial) and joints (revolute, continuous, prismatic, fixed). We learned Xacro for creating modular, maintainable models using variables and macros. We built a complete 3-DOF humanoid leg and visualized it in RViz. Proper URDF modeling is critical for accurate simulation and real-world deployment.

**Next Chapter**: We'll simulate our humanoid models in Gazebo, adding physics, sensors, and control plugins.
