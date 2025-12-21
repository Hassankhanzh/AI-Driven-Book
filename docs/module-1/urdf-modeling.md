---
sidebar_position: 4
title: 'Humanoid Modeling with URDF'
---

# Humanoid Modeling with URDF

This chapter covers the Unified Robot Description Format (URDF) for defining humanoid robot models, including links, joints, and kinematics. This prepares humanoids for both simulation and real-world control scenarios.

import RosConcept from '@site/src/components/RosConcept';
import CodeExample from '@site/src/components/CodeExample';

<RosConcept
  title="URDF"
  type="node"
  description="The Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models."
  usage="It defines the physical and visual properties of a robot, including its kinematic structure, inertial properties, visual appearance, and collision properties."
/>

## Introduction to URDF

The Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its structure, mass properties, visual appearance, and collision properties.

URDF is essential for:

- **Simulation**: Creating robot models for simulators like Gazebo
- **Visualization**: Displaying robots in tools like RViz
- **Control**: Providing structure information for robot controllers
- **Planning**: Enabling motion planning algorithms to understand robot structure

## URDF Purpose and Structure

URDF serves several important purposes in robotics:

- **Model Definition**: Defines the physical structure of a robot
- **Simulation**: Provides models for robot simulators like Gazebo
- **Visualization**: Enables 3D visualization of robots in tools like RViz
- **Kinematics**: Defines the kinematic chain for forward and inverse kinematics

A basic URDF structure includes:

- **Links**: Rigid bodies that make up the robot
- **Joints**: Connections between links that define how they move relative to each other
- **Visual Elements**: How the robot appears in visualization tools
- **Collision Elements**: How the robot interacts with the environment in simulation

## Links in URDF

Links represent rigid bodies in the robot. Each link has:

- **Name**: A unique identifier
- **Visual**: How the link appears visually
- **Collision**: How the link behaves in collision detection
- **Inertial**: Physical properties like mass and inertia

<CodeExample
  title="Simple Link Definition"
  description="A basic link definition with visual, collision, and inertial properties."
  code={`<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>`}
  language="xml"
/>

## Joints in URDF

Joints define how links connect and move relative to each other. Common joint types include:

- **Revolute**: Rotational joint with limited range
- **Continuous**: Rotational joint without limits
- **Prismatic**: Linear sliding joint with limits
- **Fixed**: No movement between links
- **Floating**: 6 DOF movement
- **Planar**: Movement on a plane

<CodeExample
  title="Joint Definition"
  description="A joint connecting two links with a specific movement type."
  code={`<joint name="base_to_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.1 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>`}
  language="xml"
/>

## Kinematics in Humanoid Models

Kinematics in humanoid robots describes the motion of the robot's links and joints without considering the forces that cause the motion. There are two types:

### Forward Kinematics
Given joint angles, calculate the position and orientation of the end effector (e.g., hand or foot).

### Inverse Kinematics
Given a desired position and orientation of the end effector, calculate the required joint angles.

For humanoid robots, kinematics are particularly important for:

- **Locomotion**: Walking, running, jumping
- **Manipulation**: Grasping objects, performing tasks
- **Balance**: Maintaining stability during movement

## Creating a Simple Humanoid Model

Here's an example of a basic humanoid torso:

<CodeExample
  title="Basic Humanoid Torso"
  description="A simple humanoid model with torso, head, and neck joint."
  code={`<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Joint connecting torso to head -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>
</robot>`}
  language="xml"
/>

## Complete Humanoid Model Example

Here's a more complete example of a humanoid model with arms and legs:

<CodeExample
  title="Complete Humanoid Model"
  description="A more complete humanoid model with torso, head, arms, and legs."
  code={`<?xml version="1.0"?>
<robot name="complete_humanoid">

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="5" velocity="1"/>
  </joint>

  <!-- Left shoulder -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.5 0.5 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left lower arm -->
  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="arm_color">
        <color rgba="0.5 0.5 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.36" effort="10" velocity="1"/>
  </joint>

  <!-- Left hip -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.2 0.8 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.08 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Left lower leg -->
  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="leg_color">
        <color rgba="0.2 0.8 0.2 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.04"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.36" upper="0" effort="20" velocity="1"/>
  </joint>

  <!-- Left foot -->
  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="foot_color">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_lower_leg"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

</robot>`}
  language="xml"
/>

## Advanced URDF Features

### Transmission Elements
Define how actuators connect to joints:

<CodeExample
  title="Transmission Definition"
  description="Defines how actuators connect to joints for control."
  code={`<transmission name="wheel_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="base_to_wheel">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>`}
  language="xml"
/>

### Gazebo-Specific Elements
Include simulation-specific properties:

<CodeExample
  title="Gazebo-Specific Properties"
  description="Simulation-specific properties for Gazebo simulator."
  code={`<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>`}
  language="xml"
/>

## URDF for Humanoid Locomotion

Humanoid robots require special considerations for locomotion. When modeling humanoid robots for walking:

- **Degrees of Freedom**: Ensure sufficient DOF in legs for stable walking
- **Foot Design**: Model feet with appropriate contact surfaces
- **Center of Mass**: Position the CoM appropriately for balance
- **Ankle Joints**: Include ankle joints for balance adjustment

## URDF for Humanoid Manipulation

For manipulation tasks, consider:

- **Arm DOF**: Ensure sufficient DOF for reaching and manipulation
- **Hand/End Effector**: Model hands or end effectors appropriately
- **Reachable Workspace**: Verify the model can reach required areas
- **Payload Capacity**: Account for carrying objects in inertial properties

## Tools for Working with URDF

Several tools help with URDF development:

- **RViz**: Visualize URDF models in 3D
- **Gazebo**: Simulate URDF models in physics environment
- **URDF Tutorials**: Official ROS tutorials for learning URDF
- **xacro**: Macro language that makes URDF more readable and maintainable

## Preparing Humanoids for Simulation and Control

When creating URDF models for simulation and control, several important considerations ensure proper behavior:

### Inertial Properties

Accurate inertial properties are important for realistic simulation and proper control:

- **Mass**: Set realistic mass values for each link based on hardware specifications
- **Center of Mass**: Position the center of mass appropriately for each link
- **Inertia Tensor**: Define the inertia tensor values that reflect how mass is distributed

<CodeExample
  title="Accurate Inertial Properties"
  description="Example of properly defined inertial properties for a robot link."
  code={`<link name="upper_arm">
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="2.5"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.001" iyy="0.01" iyz="0.0" izz="0.002"/>
  </inertial>
  <!-- visual and collision elements -->
</link>`}
  language="xml"
/>

### Collision Meshes

For efficient simulation, use simplified collision meshes:

- **Simplicity**: Use basic geometric shapes (boxes, cylinders, spheres) when possible
- **Performance**: Complex meshes slow down physics calculations
- **Accuracy**: Balance simplicity with sufficient accuracy for collision detection

### Joint Limits and Safety

Set appropriate joint limits to ensure safe operation:

- **Range of Motion**: Define realistic limits based on physical constraints
- **Effort Limits**: Specify maximum forces/torques for each joint
- **Velocity Limits**: Set maximum velocities to prevent damage

<CodeExample
  title="Joint Limits"
  description="Properly defined joint limits for safety and control."
  code={`<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  <safety_controller k_position="20" k_velocity="400" soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
</joint>`}
  language="xml"
/>

### Gazebo Configuration

For simulation in Gazebo, include physics configurations:

<CodeExample
  title="Gazebo Configuration"
  description="Gazebo-specific physics properties for realistic simulation."
  code={`<gazebo reference="upper_arm">
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
  <kp>1000000.0</kp>
  <kd>1.0</kd>
  <material>Gazebo/Blue</material>
  <self_collide>0</self_collide>
  <gravity>1</gravity>
  <max_contacts>10</max_contacts>
</gazebo>`}
  language="xml"
/>

### Transmission Definitions

For control, define transmissions that connect joints to actuators:

<CodeExample
  title="Transmission Definition"
  description="Connecting joints to control interfaces for actuator control."
  code={`<transmission name="shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="shoulder_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="shoulder_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>`}
  language="xml"
/>

### Sensor Integration

Include sensor definitions for perception capabilities:

<CodeExample
  title="Sensor Definition"
  description="Adding sensors to the robot model for perception."
  code={`<gazebo reference="head">
  <sensor name="camera" type="camera">
    <pose>0.05 0 0 0 0 0</pose>
    <camera name="head_camera">
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
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
  </sensor>
</gazebo>`}
  language="xml"
/>

## Best Practices for Humanoid URDF Models

- **Start Simple**: Begin with basic shapes and add complexity gradually
- **Use Standard Formats**: Follow established conventions for humanoid models
- **Validate Kinematics**: Test the model's kinematic chain before adding complexity
- **Check Physics**: Ensure inertial properties are physically plausible
- **Modular Design**: Structure the URDF for easy modification and reuse
- **Documentation**: Comment your URDF files to explain complex sections

## Exercises

To practice URDF modeling:

### Exercise 1: Basic Robot
Create a simple robot with a base, single rotating joint, and an arm. Visualize it in RViz.

### Exercise 2: Humanoid Leg
Model a single leg with hip, knee, and ankle joints. Verify the kinematic chain is correct.

### Exercise 3: Complete Humanoid
Extend the basic humanoid model to include arms, with shoulder, elbow, and wrist joints.

### Exercise 4: xacro Conversion
Convert your URDF to xacro format to make it more maintainable and readable.

### Exercise 5: Simulation Test
Load your humanoid model into Gazebo and test its physical properties.

## Summary

URDF is fundamental to humanoid robotics, providing the model definitions needed for simulation, visualization, and control. Properly designed URDF files enable robots to interact correctly with their environment and form the foundation for advanced robotics applications.

With this chapter, we've covered the essential components of ROS 2 for Physical AI and Humanoid Robotics. The next modules will build on these foundations with more advanced topics.