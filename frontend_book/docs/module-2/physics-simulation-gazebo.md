---
title: "Physics Simulation with Gazebo"
description: "Learn to create physics simulation environments using Gazebo for realistic robot dynamics"
sidebar_label: "Physics Simulation with Gazebo"
sidebar_position: 1
keywords: [Gazebo, physics simulation, robot dynamics, gravity, collisions, ROS2]
learning_outcomes:
  - Configure Gazebo simulation environments with proper physics parameters
  - Implement gravity and collision detection for realistic robot behavior
  - Understand robot dynamics and physics properties in simulation
  - Create realistic humanoid robot simulations in Gazebo
---

# Physics Simulation with Gazebo

## Learning Objectives

After completing this chapter, you will be able to:
- Configure Gazebo simulation environments with proper physics parameters
- Implement gravity and collision detection for realistic robot behavior
- Understand robot dynamics and physics properties in simulation
- Create realistic humanoid robot simulations in Gazebo
- Integrate Gazebo simulations with ROS 2 systems for testing and development

## Table of Contents
- [Introduction to Gazebo Physics Simulation](#introduction-to-gazebo-physics-simulation)
- [Setting Up Gazebo Environments](#setting-up-gazebo-environments)
- [Gravity and Collision Detection](#gravity-and-collision-detection)
- [Robot Dynamics and Physics Properties](#robot-dynamics-and-physics-properties)
- [Humanoid Robot Simulation in Gazebo](#humanoid-robot-simulation-in-gazebo)
- [Integrating with ROS 2](#integrating-with-ros-2)
- [Summary](#summary)

## Introduction to Gazebo Physics Simulation

Gazebo is a robotics simulation environment that provides physics simulation, sensor simulation, and realistic rendering capabilities. It is widely used in the robotics community for testing and development of robotic systems before deploying them on real robots.

Gazebo uses high-fidelity physics engines like ODE (Open Dynamics Engine), Bullet, and Simbody to simulate realistic robot behavior. This includes accurate modeling of gravity, collisions, friction, and other physical phenomena that affect robot motion and interaction with the environment.

### Key Components of Gazebo Physics Simulation
- **World files**: Define the simulation environment, including terrain, obstacles, and physics parameters
- **Model files**: Define robot models with links, joints, and visual/collision properties
- **Plugin system**: Extend simulation capabilities with custom behaviors and sensor models
- **Physics engine**: Handle the mathematical computations for realistic physics simulation

## Setting Up Gazebo Environments

### Basic Gazebo World Structure

A basic Gazebo world file defines the environment where your robot will operate. Here's an example of a minimal world file:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="digital_twin_world">
    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>

    <!-- Your robot model will be included here -->
    <!-- <include>
      <uri>model://your_robot</uri>
    </include> -->
  </world>
</sdf>
```

### Physics Engine Configuration

The physics engine configuration is critical for achieving realistic simulation behavior:

```xml
<physics type="ode">
  <!-- Maximum time step for physics updates -->
  <max_step_size>0.001</max_step_size>

  <!-- Desired speed of simulation compared to real time -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Rate at which physics updates occur -->
  <real_time_update_rate>1000.0</real_time_update_rate>

  <!-- Solver parameters for stability -->
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Gravity and Collision Detection

### Gravity Configuration

Gravity is a fundamental aspect of physics simulation that affects how objects behave in the virtual world. In Gazebo, gravity is defined globally in the world file:

```xml
<world name="my_world">
  <!-- Standard Earth gravity (9.8 m/s^2 downward) -->
  <gravity>0 0 -9.8</gravity>

  <!-- Optional: Modify gravity for special scenarios -->
  <!-- <gravity>0 0 -1.62</gravity> for lunar gravity -->

  <!-- Rest of world configuration -->
</world>
```

### Collision Detection

Collision detection in Gazebo is handled through collision geometries defined for each link in your robot model:

```xml
<link name="link_name">
  <!-- Visual representation -->
  <visual name="visual">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size>
      </box>
    </geometry>
  </visual>

  <!-- Collision geometry (used for physics simulation) -->
  <collision name="collision">
    <geometry>
      <box>
        <size>0.1 0.1 0.1</size>
      </box>
    </geometry>
    <!-- Surface parameters affecting contact behavior -->
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>
          <mu2>1.0</mu2>
        </ode>
      </friction>
      <bounce>
        <restitution_coefficient>0.0</restitution_coefficient>
        <threshold>100000.0</threshold>
      </bounce>
      <contact>
        <ode>
          <soft_cfm>0.0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1000000000000.0</kp>
          <kd>1.0</kd>
          <max_vel>100.0</max_vel>
          <min_depth>0.001</min_depth>
        </ode>
      </contact>
    </surface>
  </collision>
</link>
```

## Robot Dynamics and Physics Properties

### Mass and Inertia

Accurate mass and inertia properties are crucial for realistic robot dynamics:

```xml
<link name="link_name">
  <!-- Inertial properties affecting dynamics -->
  <inertial>
    <!-- Mass in kilograms -->
    <mass>1.0</mass>

    <!-- Inertia matrix (diagonal elements for simple shapes) -->
    <inertia>
      <ixx>0.001</ixx>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyy>0.001</iyy>
      <iyz>0.0</iyz>
      <izz>0.001</izz>
    </inertia>
  </inertial>

  <!-- Visual and collision geometries -->
</link>
```

### Joint Dynamics

Joints connect different links of your robot and can have dynamic properties:

```xml
<joint name="joint_name" type="revolute">
  <parent>parent_link</parent>
  <child>child_link</child>

  <!-- Joint axis and limits -->
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100.0</effort>
      <velocity>1.0</velocity>
    </limit>
    <!-- Dynamics parameters -->
    <dynamics>
      <damping>0.1</damping>
      <friction>0.0</friction>
    </dynamics>
  </axis>
</joint>
```

## Humanoid Robot Simulation in Gazebo

### Creating a Simple Humanoid Model

Here's an example of a simplified humanoid robot model that can be used in Gazebo:

```xml
<?xml version="1.0" ?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Base/Torso link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Similar definitions for arms, legs, etc. -->
</robot>
```

## Integrating with ROS 2

### ROS 2 Control in Gazebo

To control your simulated robot with ROS 2, you'll need to use the Gazebo ROS 2 control interface:

```xml
<!-- In your robot's URDF/XACRO file -->
<xacro:macro name="gazebo_ros_control" params="prefix">
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>$(arg robot_namespace)</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
      <legacyModeNS>true</legacyModeNS>
    </plugin>
  </gazebo>
</xacro:macro>
```

### Launching Gazebo with ROS 2

Example launch file to start Gazebo with your robot:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with world
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                      '-entity', 'my_robot'],
            output='screen'
        ),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'use_sim_time': True}]
        )
    ])
```

## Summary

Gazebo provides a powerful physics simulation environment that enables realistic testing and development of robotic systems. By properly configuring gravity, collision detection, and robot dynamics, you can create digital twins that accurately represent the behavior of real robots. The integration with ROS 2 allows you to test your robot's control algorithms in a safe, virtual environment before deploying them on physical hardware.

## Next Steps

Continue with the next chapters in this module:

- [High-Fidelity Environments with Unity](./high-fidelity-unity) - Learn how to create realistic visual environments with Unity
- [Sensor Simulation](./sensor-simulation) - Understand how to simulate various sensors for perception testing