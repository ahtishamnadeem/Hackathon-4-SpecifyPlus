# Quickstart: Setting up Digital Twin Simulations

## Prerequisites

- Basic understanding of ROS/ROS2 (from Module 1)
- Computer capable of running physics simulations (4+ GB RAM recommended)
- Gazebo Classic or Garden installation
- Unity Hub and Unity Editor (optional, for Unity content)
- Git for version control
- A code editor (VS Code recommended)

## Installation Steps for Gazebo Simulation

### 1. Install Gazebo
```bash
# For Ubuntu/Debian
sudo apt-get update
sudo apt-get install gazebo

# For ROS2 users
sudo apt-get install ros-<ros2-distro>-gazebo-*
```

### 2. Verify Installation
```bash
gazebo --version
```

### 3. Test Basic Simulation
```bash
gazebo
```

## Installation Steps for Unity Simulation

### 1. Install Unity Hub
- Download Unity Hub from https://unity.com/
- Install and create an account

### 2. Install Unity Editor
- Open Unity Hub
- Install the latest LTS version of Unity Editor

### 3. Install Unity Robotics packages
- In Unity Hub, go to the Packages tab
- Install Unity Robotics Hub and related packages

## Getting Started with Digital Twin Concepts

### 1. Understand the Digital Twin Architecture
- Physical system (real robot)
- Virtual system (simulation model)
- Communication layer (data synchronization)
- Visualization layer (monitoring and control)

### 2. Create Your First Simulation Environment
1. Start with a simple Gazebo world
2. Add a basic robot model
3. Configure physics properties
4. Add sensor plugins
5. Connect to ROS2 for data exchange

### 3. Basic Gazebo World File
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="digital_twin_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- Add your robot model here -->
  </world>
</sdf>
```

## Building for Production

To build the Docusaurus site for deployment:

```bash
npm run build
# or
yarn build
```

The built site will be available in the `build/` directory.

## Troubleshooting

### Common Issues

1. **Gazebo won't start**
   - Error: `Error [ResourceManager.cc:60] Unable to find resource path`
   - Solution: Check GAZEBO_MODEL_PATH environment variable

2. **Physics simulation is unstable**
   - Error: Robot behaves erratically in simulation
   - Solution: Adjust solver parameters and time step in world file

3. **Unity installation issues**
   - Error during package installation
   - Solution: Check system requirements and internet connection

4. **ROS2 connection problems**
   - Error: Cannot connect simulation to ROS2
   - Solution: Verify ROS_DOMAIN_ID and network configuration