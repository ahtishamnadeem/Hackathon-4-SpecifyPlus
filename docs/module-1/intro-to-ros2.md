---
title: "Introduction to ROS 2 and Robotic Middleware"
description: "Learn the fundamentals of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment"
sidebar_label: "Introduction to ROS 2"
sidebar_position: 1
keywords: [ROS 2, robotic middleware, nodes, topics, services, DDS]
learning_outcomes:
  - Explain the role of ROS 2 in Physical AI and humanoid robotics
  - Identify and describe the core components of ROS 2: nodes, topics, services, and message passing
  - Understand real-time constraints and DDS concepts in robotics
  - Recognize how ROS 2 enables communication between different parts of a robotic system
---

# Introduction to ROS 2 and Robotic Middleware

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the role of ROS 2 in Physical AI and humanoid robotics
- Identify and describe the core components of ROS 2: nodes, topics, services, and message passing
- Understand real-time constraints and DDS concepts in robotics
- Recognize how ROS 2 enables communication between different parts of a robotic system

## Table of Contents
- [The Role of ROS 2 in Physical AI](#the-role-of-ros2-in-physical-ai)
- [Core Concepts: Nodes, Topics, Services](#core-concepts-nodes-topics-services)
- [Message Passing in ROS 2](#message-passing-in-ros2)
- [Real-time Constraints and DDS](#real-time-constraints-and-dds)
- [Summary](#summary)

## The Role of ROS 2 in Physical AI

Robot Operating System 2 (ROS 2) serves as the "nervous system" of robotic platforms, enabling different software components to communicate with each other seamlessly. In the context of Physical AI and humanoid robotics, ROS 2 provides the infrastructure for:

- **Coordination**: Allowing multiple subsystems (vision, control, planning) to work together
- **Communication**: Enabling data exchange between different parts of the robot
- **Integration**: Providing standardized interfaces for hardware and software components
- **Scalability**: Supporting both simple and complex robotic systems

ROS 2 is particularly well-suited for humanoid robotics because it handles the complexity of managing multiple sensors, actuators, and computational nodes that need to work in real-time coordination.

## Core Concepts: Nodes, Topics, Services

### Nodes

A **node** is a process that performs computation in the ROS 2 system. Nodes are the fundamental building blocks of a ROS 2 application. Each node typically performs a specific task, such as:

- Processing sensor data
- Controlling actuators
- Planning robot movements
- Managing robot state

Nodes are designed to be modular and reusable, allowing complex systems to be built from simple, focused components.

### Topics

**Topics** provide a way for nodes to send and receive data asynchronously using a publish/subscribe pattern. Key characteristics of topics include:

- **Decoupling**: Publishers and subscribers don't need to know about each other
- **Broadcasting**: One publisher can send data to multiple subscribers
- **Real-time**: Data flows continuously from publishers to subscribers
- **Typed messages**: All data is structured according to defined message types

### Services

**Services** enable synchronous request/response communication between nodes. Services are appropriate for:

- Actions that need confirmation
- Requesting specific data
- Operations that should complete before continuing
- Synchronous interactions where timing matters

## Message Passing in ROS 2

ROS 2 uses a robust message passing system that allows nodes to communicate through topics, services, and actions. The message passing system includes:

### Publishers and Subscribers

```python
# Example of a simple publisher in Python using rclpy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
```

### Message Types

ROS 2 provides standard message types and allows users to define custom message types. Common message types include:

- `std_msgs`: Basic data types (integers, floats, strings, etc.)
- `geometry_msgs`: Geometric primitives (points, poses, transforms)
- `sensor_msgs`: Sensor data (images, laser scans, joint states)
- `nav_msgs`: Navigation-related messages (paths, occupancy grids)

## Real-time Constraints and DDS

### Data Distribution Service (DDS)

ROS 2 uses DDS (Data Distribution Service) as its underlying communication middleware. DDS provides:

- **Quality of Service (QoS)**: Configurable reliability, durability, and liveliness settings
- **Discovery**: Automatic discovery of nodes and their communication interfaces
- **Real-time performance**: Low-latency, deterministic communication
- **Scalability**: Support for large, distributed robotic systems

### QoS Profiles

QoS profiles allow fine-tuning of communication behavior:

- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local data
- **History**: Keep all samples vs. keep last N samples
- **Deadline**: Maximum time between consecutive samples

## Summary

ROS 2 serves as the foundational middleware for robotic systems, providing the communication infrastructure that allows different components to work together. Understanding the core concepts of nodes, topics, and services is essential for developing robotic applications, particularly in the context of Physical AI and humanoid robotics where real-time coordination is critical.

## Next Steps

Continue with the next chapters in this module:

- [Python Agents and ROS 2 Control](./python-agents-ros2.md) - Learn how to bridge Python AI agents to robot controllers
- [Humanoid Robot Modeling with URDF](./humanoid-urdf-modeling.md) - Learn how to model humanoid robots using URDF