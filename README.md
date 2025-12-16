# ROS 2 Fundamentals for Physical AI and Humanoid Robotics

This educational book provides comprehensive coverage of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment. It is designed for AI and robotics students with basic Python knowledge entering Physical AI and humanoid robotics.

## Overview

This book covers three main modules:

1. **Introduction to ROS 2 and Robotic Middleware** - Learn the fundamentals of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment
2. **Python Agents and ROS 2 Control** - Learn how to bridge Python AI agents to robot controllers using ROS 2
3. **Humanoid Robot Modeling with URDF** - Learn how to model humanoid robots using URDF for simulation and control

## Learning Objectives

After completing this course, students will be able to:
- Understand the role of ROS 2 in Physical AI and humanoid robotics
- Use rclpy to build ROS 2 nodes that interface with Python AI agents
- Create URDF models with proper links, joints, and coordinate frames
- Implement publishers, subscribers, services, and actions in ROS 2
- Prepare humanoid robots for simulation and control

## Prerequisites

- Basic Python knowledge
- Understanding of fundamental programming concepts
- Interest in robotics and AI

## Installation

```bash
npm install
```

## Local Development

```bash
npm start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
npm build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true npm run deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.

## Structure

The content is organized as follows:
- `/docs/module-1/` - Contains all content for Module 1: The Robotic Nervous System
  - `intro-to-ros2.md` - Introduction to ROS 2 and Robotic Middleware
  - `python-agents-ros2.md` - Python Agents and ROS 2 Control
  - `humanoid-urdf-modeling.md` - Humanoid Robot Modeling with URDF

## Contributing

For more information, see the [Docusaurus documentation](https://docusaurus.io/).
