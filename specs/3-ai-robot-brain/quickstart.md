# Quickstart: Setting up AI-Robot Brain Development Environment

## Prerequisites

- Basic understanding of ROS/ROS2 (from Module 1)
- Computer with NVIDIA GPU (CUDA-compatible, ideally RTX series)
- NVIDIA GPU drivers installed (version supporting CUDA 11.8+)
- Isaac Sim installed (compatible with your GPU)
- Isaac ROS packages installed
- Nav2 navigation stack installed
- Git for version control
- A code editor (VS Code recommended)

### Hardware Requirements
- NVIDIA GPU with Tensor Cores (RTX 3060 or better recommended)
- At least 8GB GPU memory for Isaac Sim
- 16GB+ system RAM recommended
- SSD storage for faster asset loading

### Software Requirements
- Ubuntu 20.04 LTS or 22.04 LTS (recommended for Isaac ecosystem)
- CUDA Toolkit 11.8 or later
- cuDNN library
- Isaac Sim (Part of Isaac ROS Developer Kit)
- Isaac ROS packages
- ROS 2 Humble Hawksbill or Rolling Ridley

## Installation Steps

### 1. Verify GPU and Driver Compatibility
```bash
nvidia-smi
nvcc --version  # Verify CUDA installation
```

### 2. Install Isaac Sim
```bash
# Follow official Isaac Sim installation guide:
# https://docs.nvidia.com/isaac/isaacl_sim/index.html
# Download from NVIDIA Developer website
# Install using the provided installer
```

### 3. Install Isaac ROS Packages
```bash
# Add NVIDIA package repositories
sudo apt update
sudo apt install software-properties-common
wget https://repo.download.nvidia.com/nvidia.pub
sudo apt-key add nvidia.pub
echo 'deb https://repo.download.nvidia.com/ $(lsb_release -cs)/main' | sudo tee /etc/apt/sources.list.d/nvidia-isaac.list

# Install Isaac ROS packages
sudo apt update
sudo apt install nvidia-isaac-ros-dev-isolated
```

### 4. Install Nav2
```bash
# If not already installed with ROS 2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

### 5. Clone and Setup Documentation Repository
```bash
git clone <repository-url>
cd <repository-name>
npm install  # Install Docusaurus dependencies
```

### 6. Start Development Server
```bash
npm run start
# Navigate to http://localhost:3000 to see the documentation
```

## Getting Started with Isaac Sim

### 1. Launch Isaac Sim
```bash
# Navigate to Isaac Sim installation directory
./isaac-sim.sh
```

### 2. Basic Scene Setup
1. Open Isaac Sim
2. Create a new scene or load an existing one
3. Add a humanoid robot model (e.g., ATRIAS, ATLAS, or custom model)
4. Configure lighting and environment settings
5. Verify physics properties are properly set

### 3. Synthetic Data Generation
1. Configure sensors on your robot (cameras, LiDAR, IMUs)
2. Set up annotation assets for synthetic data generation
3. Run simulation and capture data
4. Export data in formats suitable for training (PNG, JSON, etc.)

## Getting Started with Isaac ROS Pipelines

### 1. Basic Isaac ROS Pipeline
```bash
# Terminal 1: Launch Isaac ROS bridge
ros2 launch isaac_ros_bridge isaac_ros_bridge.launch.py

# Terminal 2: Launch your perception pipeline
ros2 launch isaac_ros_perceptor_vslam vslam_pipeline.launch.py
```

### 2. Performance Monitoring
```bash
# Monitor GPU utilization
nvidia-smi -l 1

# Monitor ROS 2 topics
ros2 topic echo /camera/color/image_rect_color
```

## Getting Started with Nav2 for Humanoid Robots

### 1. Configure Nav2 for Humanoid
```bash
# Create custom Nav2 configuration for humanoid robots
# Example parameters for bipedal navigation:
# - Footstep planner parameters
# - Balance constraint settings
# - COM (Center of Mass) control parameters
```

### 2. Launch Humanoid Navigation
```bash
# Launch Nav2 with humanoid-specific configuration
ros2 launch nav2_bringup navigation_launch.py \
  namespace:=humanoid_robot \
  use_sim_time:=true \
  params_file:=install/nav2_humangoid_params.yaml
```

## Building for Production

To build the Docusaurus site for deployment:

```bash
npm run build
# or
yarn build
```

The built site will be available in the `build/` directory.

## Troubleshooting Common Issues

### Isaac Sim Issues
1. **Isaac Sim won't start**
   - Error: "Could not initialize GLX"
   - Solution: Check NVIDIA driver installation and X11 forwarding if using remote access

2. **Low rendering performance**
   - Error: Frame rate below acceptable threshold
   - Solution: Check GPU memory usage and reduce scene complexity

3. **Physics simulation instability**
   - Error: Robot behaves erratically in simulation
   - Solution: Adjust solver parameters and time step in simulation settings

### Isaac ROS Issues
1. **Hardware acceleration not working**
   - Error: Isaac ROS nodes running on CPU instead of GPU
   - Solution: Verify CUDA installation and Isaac ROS package versions

2. **Pipeline performance below expectations**
   - Error: Processing slower than real-time
   - Solution: Profile bottlenecks and optimize pipeline configuration

### Nav2 Issues
1. **Humanoid navigation fails**
   - Error: Path planner unable to find valid path for bipedal robot
   - Solution: Adjust costmap parameters and add humanoid-specific constraints

2. **Balance maintenance issues**
   - Error: Robot falls during navigation
   - Solution: Implement additional balance control layer