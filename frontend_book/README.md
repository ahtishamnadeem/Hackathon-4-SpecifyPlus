# ROS 2 Fundamentals for Physical AI and Humanoid Robotics

This educational book provides comprehensive coverage of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment. It is designed for AI and robotics students with basic Python knowledge entering Physical AI and humanoid robotics.

## Overview

This book covers multiple modules:

1. **Module 1: Introduction to ROS 2 and Robotic Middleware** - Learn the fundamentals of ROS 2 as the robotic nervous system for humanoid control, communication, and embodiment
2. **Module 2: The Digital Twin** - Learn about digital twins for humanoid robots using physics-based and interactive simulations including Gazebo physics simulation, Unity environments, and sensor simulation
3. **Module 3: The AI-Robot Brain** - Learn to implement advanced AI capabilities in humanoid robots using NVIDIA Isaac and ROS-based tools including physics simulation with Isaac Sim, hardware-accelerated AI with Isaac ROS, and navigation with Nav2 for humanoid robots
4. **Module 4: Vision-Language-Action** - Learn to integrate voice processing, cognitive planning with LLMs, and action execution to create autonomous humanoid systems that respond to voice commands with complex behaviors

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
