---
title: "Humanoid Robot Modeling with URDF"
description: "Learn how to model humanoid robots using URDF for simulation and control"
sidebar_label: "Humanoid Robot Modeling with URDF"
sidebar_position: 3
keywords: [URDF, humanoid robots, links, joints, sensors, coordinate frames]
learning_outcomes:
  - Understand the purpose of URDF in humanoid robots
  - Create URDF models with proper links, joints, and coordinate frames
  - Include sensors in URDF models for humanoid robots
  - Prepare humanoid robots for simulation and control
---

# Humanoid Robot Modeling with URDF

## Learning Objectives

After completing this chapter, you will be able to:
- Understand the purpose of URDF in humanoid robots
- Create URDF models with proper links, joints, and coordinate frames
- Include sensors in URDF models for humanoid robots
- Prepare humanoid robots for simulation and control

## Table of Contents
- [Purpose of URDF in Humanoid Robots](#purpose-of-urdf-in-humanoid-robots)
- [Links, Joints, Sensors, and Coordinate Frames](#links-joints-sensors-and-coordinate-frames)
- [Creating a Basic Humanoid URDF Model](#creating-a-basic-humanoid-urdf-model)
- [Adding Sensors to URDF Models](#adding-sensors-to-urdf-models)
- [Preparing Humanoids for Simulation and Control](#preparing-humanoids-for-simulation-and-control)
- [Summary](#summary)

## Purpose of URDF in Humanoid Robots

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF serves several critical purposes:

- **Physical Description**: Defines the physical structure of the robot including links, joints, and their relationships
- **Visual Representation**: Specifies how the robot should be visualized in simulation and visualization tools
- **Collision Models**: Defines collision geometry for physics simulation
- **Kinematic Chain**: Establishes the kinematic structure necessary for forward and inverse kinematics
- **Sensor Integration**: Specifies where sensors are mounted on the robot

URDF is fundamental to humanoid robotics because it provides the necessary information for:
- Robot simulation in environments like Gazebo
- Robot visualization in tools like RViz
- Motion planning algorithms
- Control systems that need to understand robot structure

## Links, Joints, Sensors, and Coordinate Frames

### Links

A **link** represents a rigid part of the robot. In a humanoid robot, links typically represent:

- Torso/trunk
- Head
- Arms (upper arm, lower arm, hand)
- Legs (thigh, shin, foot)
- Individual fingers (optional)

Each link has properties including:
- **Mass**: The mass of the link
- **Inertia**: The inertial properties of the link
- **Visual**: How the link appears visually
- **Collision**: How the link behaves in collision detection

### Joints

A **joint** connects two links and defines how they can move relative to each other. Common joint types in humanoid robots:

- **Revolute**: Rotational joint with limited range of motion (like an elbow)
- **Continuous**: Rotational joint with unlimited range of motion
- **Prismatic**: Linear sliding joint
- **Fixed**: No movement between links (for attaching sensors or decorations)

### Coordinate Frames

URDF establishes a tree structure of coordinate frames where:
- Each link has its own coordinate frame
- Joint transformations define how frames relate to each other
- The base frame is typically the robot's main body or a fixed reference point
- All other frames are defined relative to their parent frame

### Sensors

Sensors in URDF are typically modeled as:
- Fixed joints attaching the sensor to a link
- Special sensor plugins for simulation
- Proper coordinate frame definitions for sensor data interpretation

## Creating a Basic Humanoid URDF Model

Here's an example of a simple humanoid URDF model:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base/Root link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.008" ixy="0.0" ixz="0.0" iyy="0.008" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.1 0.1 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 -0.1 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="0.5" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Adding Sensors to URDF Models

Sensors are typically added to URDF models as fixed joints attaching the sensor to a link:

```xml
<!-- Example: IMU sensor attached to the head -->
<link name="imu_link">
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
  </inertial>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="head"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

## Preparing Humanoids for Simulation and Control

### Simulation Considerations

When preparing URDF models for simulation:

1. **Proper Inertial Properties**: Ensure mass and inertia values are realistic
2. **Collision Geometry**: Use appropriate collision shapes for performance
3. **Joint Limits**: Set realistic limits based on physical constraints
4. **Transmission Elements**: Define how joints connect to actuators

### Control Considerations

For control systems:

1. **Kinematic Chains**: Ensure proper parent-child relationships
2. **Frame Definitions**: Use consistent coordinate frame conventions
3. **Joint Types**: Select appropriate joint types for desired motion
4. **Sensor Integration**: Position sensors correctly for control algorithms

### Example: Transmission Definition

```xml
<!-- Example transmission for a joint -->
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Summary

URDF is essential for humanoid robotics as it provides the complete description of a robot's physical structure, including links, joints, sensors, and coordinate frames. Creating proper URDF models is crucial for both simulation and control of humanoid robots. The examples in this chapter demonstrate the fundamental elements needed to create a humanoid robot model that can be used in ROS-based systems.

With a properly defined URDF model, humanoid robots can be simulated, visualized, and controlled effectively within the ROS ecosystem.

## Next Steps

Continue exploring ROS 2 concepts:

- [Previous: Introduction to ROS 2](./intro-to-ros2.md) - Review fundamental ROS 2 concepts
- [Previous: Python Agents and ROS 2 Control](./python-agents-ros2.md) - Review Python agent integration