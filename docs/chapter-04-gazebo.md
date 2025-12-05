---
sidebar_position: 5
---

# Chapter 4: Gazebo Simulation - Testing in the Digital Twin

## Learning Objectives

- Understand the role of simulation in robotics development
- Set up Gazebo for humanoid robot simulation
- Spawn URDF models in Gazebo worlds
- Add physics properties and sensor plugins
- Implement control plugins for actuators
- Debug common simulation issues

## Why Simulation?

Testing on physical hardware is:
- **Dangerous**: Falling robots can break
- **Slow**: Limited to real-time
- **Expensive**: Hardware wear and tear
- **Limited**: Can't test edge cases easily

Simulation enables:
- **Rapid iteration**: Test thousands of scenarios overnight
- **Safety**: No risk of hardware damage
- **Scalability**: Train multiple robots in parallel
- **Reproducibility**: Exact same conditions every time

### Gazebo vs. Gazebo Classic

| Feature | Gazebo Classic | Gazebo (Ignition/New) |
|---------|----------------|------------------------|
| Physics Engines | ODE, Bullet, Simbody | DART, TPE |
| Rendering | OGRE 1.x | OGRE 2.x, Optix |
| Performance | Moderate | Better |
| ROS Integration | Native (ROS 1) | ros_gz_bridge (ROS 2) |
| Status | Deprecated (2025) | Active development |

**We use Gazebo (new)** for this course.

## Installing Gazebo

```bash
# Install Gazebo Fortress (compatible with ROS 2 Humble)
sudo apt-get update
sudo apt-get install ros-humble-ros-gz

# Verify installation
gz sim --version
```

## Spawning a Robot in Gazebo

### Step 1: Create a World File

Create `worlds/empty_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="empty_world">
    
    <!-- Physics -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

### Step 2: Launch File to Spawn Robot

Create `launch/gazebo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('humanoid_leg_description')
    
    # Gazebo server
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r empty_world.sdf'}.items()
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_leg',
            '-topic', 'robot_description',
            '-z', '1.0'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': open(os.path.join(pkg_dir, 'urdf', 'leg.urdf')).read()
        }]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

## Adding Physics Properties

### Friction and Contact

```xml
<collision name="foot_collision">
  <geometry>
    <box size="0.2 0.1 0.05"/>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>  <!-- Coefficient of friction -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1000000.0</kp>  <!-- Contact stiffness -->
        <kd>100.0</kd>       <!-- Contact damping -->
      </ode>
    </contact>
  </surface>
</collision>
```

### Joint Dynamics

```xml
<joint name="knee" type="revolute">
  <parent link="thigh"/>
  <child link="shank"/>
  <axis xyz="0 1 0">
    <dynamics>
      <damping>0.7</damping>
      <friction>0.5</friction>
    </dynamics>
  </axis>
  <limit lower="0" upper="2.35" effort="100" velocity="3.0"/>
</joint>
```

## Sensor Plugins

### IMU (Inertial Measurement Unit)

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </x>
      <!-- y and z similar -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.1</stddev>
        </noise>
      </x>
      <!-- y and z similar -->
    </linear_acceleration>
  </imu>
  <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>~/out:=imu/data</remapping>
    </ros>
  </plugin>
</sensor>
```

### Camera

```xml
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin filename="libgazebo_ros_camera.so" name="camera_controller">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>~/image_raw:=camera/image_raw</remapping>
    </ros>
  </plugin>
</sensor>
```

### LiDAR

```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.26</min_angle>
        <max_angle>0.26</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </lidar>
  <plugin filename="libgazebo_ros_ray_sensor.so" name="lidar_controller">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
  </plugin>
</sensor>
```

## Control Plugins

### Joint Position Controller

```xml
<plugin filename="libgazebo_ros_joint_state_publisher.so" name="joint_state_publisher">
  <ros>
    <namespace>/humanoid</namespace>
    <remapping>~/out:=joint_states</remapping>
  </ros>
  <update_rate>50</update_rate>
  <joint_name>hip</joint_name>
  <joint_name>knee</joint_name>
  <joint_name>ankle</joint_name>
</plugin>

<plugin filename="libgazebo_ros_joint_pose_trajectory.so" name="joint_trajectory_plugin">
  <ros>
    <namespace>/humanoid</namespace>
    <remapping>~/set_joint_trajectory:=joint_trajectory</remapping>
  </ros>
  <update_rate>100</update_rate>
</plugin>
```

### Differential Drive (for mobile base)

```xml
<plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
  <update_rate>50</update_rate>
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.2</wheel_diameter>
  <max_wheel_torque>20</max_wheel_torque>
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
  <odometry_frame>odom</odometry_frame>
  <robot_base_frame>base_link</robot_base_frame>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
</plugin>
```

## Lab Exercise: Simulating Gravity and Balance

### Objective
Spawn a humanoid leg in Gazebo and observe:
1. Does it fall over due to gravity?
2. Can you apply joint torques to prevent falling?

### Step 1: Add Gravity to World

Already included in `empty_world.sdf` by default (9.81 m/s²).

### Step 2: Launch Simulation

```bash
ros2 launch humanoid_leg_description gazebo.launch.py
```

### Step 3: Apply Joint Torque

```bash
# Publish joint trajectory to move the hip
ros2 topic pub /humanoid/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['hip'],
  points: [
    {positions: [0.5], time_from_start: {sec: 1}}
  ]
}"
```

### Step 4: Observe

- Does the leg swing?
- Does it respect joint limits?
- What happens if you set an impossible position?

## Debugging Common Issues

### Robot Falls Through Ground

**Cause**: Missing or incorrect collision geometry.

**Solution**: Ensure all links have `<collision>` elements.

### Robot Explodes or Vibrates

**Cause**: Incorrect inertia values or too-high contact stiffness.

**Solution**:
- Use realistic inertia (tools: MeshLab, SolidWorks)
- Reduce `kp` (contact stiffness) to 10,000-100,000

### Simulation Runs Slowly

**Cause**: Complex collision meshes or too many contacts.

**Solution**:
- Use primitive shapes for collision
- Reduce physics update rate (but maintain stability)

### Joints Don't Move

**Cause**: Missing control plugin or incorrect topic names.

**Solution**: Check `ros2 topic list` and verify plugin configuration.

## Quiz

1. **What is the primary benefit of simulation over hardware testing?**
   - A) More realistic
   - B) Faster iteration and safety
   - C) Cheaper sensors
   - D) Better graphics
   
   **Answer: B**

2. **Which physics engine is used in Gazebo (new)?**
   - A) ODE
   - B) Bullet
   - C) DART
   - D) PhysX
   
   **Answer: C**

3. **What does the `<mu>` parameter control in collision surfaces?**
   - A) Mass
   - B) Friction coefficient
   - C) Damping
   - D) Elasticity
   
   **Answer: B**

4. **Why use simplified collision geometry?**
   - A) Looks better
   - B) Easier to model
   - C) Faster simulation
   - D) Required by Gazebo
   
   **Answer: C**

## Summary

In this chapter, we learned to simulate humanoid robots in Gazebo. We covered world creation, robot spawning, physics properties (friction, damping), sensor plugins (IMU, camera, LiDAR), and control plugins. Simulation is essential for safe, rapid development before deploying to hardware.

**Next Chapter**: We'll explore Unity for high-fidelity visualization and human-robot interaction scenarios.
