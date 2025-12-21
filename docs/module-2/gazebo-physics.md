---
sidebar_position: 2
title: 'Physics Simulation with Gazebo'
---

# Physics Simulation with Gazebo

This chapter covers physics simulation using Gazebo, including gravity, collisions, and humanoid dynamics for digital twin applications. Gazebo is a powerful physics simulation environment that provides realistic modeling of physical interactions for humanoid robots.

import SimulationConcept from '@site/src/components/SimulationConcept';
import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives objectives={[
  "Configure gravity parameters in Gazebo for realistic physics simulation",
  "Set up collision detection for accurate physical interactions",
  "Model humanoid dynamics for realistic movement patterns",
  "Implement practical examples demonstrating physics simulation",
  "Apply physics accuracy requirements for digital twin applications"
]} />

## Introduction to Gazebo Physics

Gazebo is a 3D simulation environment that provides realistic physics simulation capabilities for robotics applications. It's widely used in the robotics community for testing algorithms, training robots, and developing digital twins.

<SimulationConcept
  title="Gazebo Physics Engine"
  type="gazebo"
  description="Gazebo uses the Open Dynamics Engine (ODE), Bullet, or DART physics engines to simulate realistic physics interactions."
  usage="Essential for creating accurate digital twins that behave like real-world robots."
/>

## Gravity Configuration in Gazebo

Gravity is a fundamental parameter that affects how all objects behave in the simulation. In Gazebo, the default gravity is set to Earth's gravity (-9.81 m/s² in the Z direction), but this can be customized for different scenarios.

### Setting Global Gravity

Global gravity affects all objects in the simulation world. It's typically configured in the world file:

```xml
<sdf version='1.6'>
  <world name='default'>
    <gravity>0 0 -9.8</gravity>
    <!-- Other world elements -->
  </world>
</sdf>
```

### Custom Gravity for Specific Models

For specialized scenarios, you can adjust gravity for specific models or implement custom physics parameters:

```xml
<model name='humanoid_robot'>
  <link name='base_link'>
    <gravity>1</gravity> <!-- Enable/disable gravity for this link -->
    <!-- Other link properties -->
  </link>
</model>
```

### Gravity Considerations for Humanoid Robots

When simulating humanoid robots, consider these gravity-related factors:

- **Stability**: Proper gravity ensures realistic balance and walking patterns
- **Joint forces**: Gravity affects the forces experienced by actuators
- **Center of mass**: Accurate gravity simulation helps maintain realistic CoM calculations

## Collision Detection Setup

Collision detection is crucial for preventing objects from passing through each other and for detecting contacts between robot parts and the environment.

### Collision vs Visual Elements

In Gazebo, models have separate collision and visual elements:

- **Visual**: Defines how the object appears (color, texture, shape)
- **Collision**: Defines the physical properties (shape, friction, contact behavior)

```xml
<link name='link_with_visual_and_collision'>
  <!-- Visual element -->
  <visual name='visual'>
    <geometry>
      <box>
        <size>1 1 1</size>
      </box>
    </geometry>
    <material>
      <ambient>0.8 0.8 0.8 1</ambient>
    </material>
  </visual>

  <!-- Collision element -->
  <collision name='collision'>
    <geometry>
      <box>
        <size>1 1 1</size>
      </box>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

### Collision Types

Gazebo supports several collision geometries:

- **Box**: Rectangular prisms
- **Sphere**: Spherical objects
- **Cylinder**: Cylindrical shapes
- **Mesh**: Complex custom shapes
- **Plane**: Infinite flat surfaces

### Contact Sensors

For more detailed collision information, you can add contact sensors to detect when and where collisions occur:

```xml
<sensor name='contact_sensor' type='contact'>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <contact>
    <collision>collision_name</collision>
  </contact>
  <plugin name='contact_plugin' filename='libgazebo_ros_bumper.so'>
    <alwaysOn>true</alwaysOn>
    <updateRate>100.0</updateRate>
    <bumperTopicName>bumper_state</bumperTopicName>
  </plugin>
</sensor>
```

## Humanoid Dynamics Modeling

Humanoid dynamics modeling involves creating accurate representations of how humanoid robots move and interact with their environment, considering factors like joint constraints, center of mass, and actuator dynamics.

### Joint Constraints and Limits

Properly configured joint limits are essential for realistic humanoid movement:

```xml
<joint name="hip_joint" type="revolute">
  <parent>torso</parent>
  <child>thigh</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>  <!-- -90 degrees -->
      <upper>1.57</upper>   <!-- 90 degrees -->
      <effort>100</effort>
      <velocity>3.14</velocity>
    </limit>
  </axis>
</joint>
```

### Center of Mass Considerations

The center of mass significantly affects the stability and movement of humanoid robots:

- **Accurate CoM**: Should match the physical robot's center of mass
- **Dynamic CoM**: Changes based on the robot's posture and carried loads
- **Balance control**: Critical for maintaining stable locomotion

### Inertial Properties

Inertial properties define how mass is distributed in each link:

```xml
<inertial>
  <mass>1.0</mass>
  <inertia>
    <ixx>0.01</ixx>
    <ixy>0.0</ixy>
    <ixz>0.0</ixz>
    <iyy>0.01</iyy>
    <iyz>0.0</iyz>
    <izz>0.01</izz>
  </inertia>
</inertial>
```

## Physics Accuracy Requirements

To ensure your digital twin accurately represents the physical robot:

- **Parameter validation**: Compare simulation behavior with real-world robot performance
- **Sensor simulation**: Match real sensor characteristics and noise patterns
- **Actuator modeling**: Include realistic motor dynamics and limitations
- **Environmental factors**: Account for friction, air resistance, and other forces

## Practical Examples

### Example 1: Simple Humanoid Model with Physics Properties

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_humanoid">
    <link name="base_link">
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.1</iyy>
          <iyz>0.0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.3 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
        </material>
      </visual>

      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.3 0.3</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Example 2: Physics Parameter Configuration

```bash
# Launch Gazebo with custom gravity
gazebo -sdf -world-file custom_gravity.world

# Or programmatically through ROS 2
ros2 service call /world/set_gravity gazebo_msgs/srv/SetGravity "{gravity: {x: 0, y: 0, z: -9.8}}"
```

## Exercises

### Exercise 1: Gravity Adjustment
Create a simulation where you adjust the gravity from Earth's value (-9.81 m/s²) to the Moon's value (-1.62 m/s²). Observe how this affects the movement and stability of a humanoid robot.

### Exercise 2: Collision Detection
Design a simple humanoid robot model with appropriate collision geometries for each link. Test the collision detection by having the robot interact with objects in the environment.

### Exercise 3: Dynamics Tuning
Implement a simple walking pattern for a humanoid robot and tune the inertial properties to achieve stable locomotion.

## Summary

Physics simulation in Gazebo forms the foundation of accurate digital twin implementations. By properly configuring gravity, collision detection, and humanoid dynamics, you can create simulations that closely match real-world behavior. This accuracy is essential for developing and testing humanoid robot algorithms before deploying them on physical hardware.