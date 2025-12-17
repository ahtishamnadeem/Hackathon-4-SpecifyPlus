---
title: "Hardware-Accelerated AI with Isaac ROS"
description: "Learn to implement Isaac ROS for hardware-accelerated AI processing with focus on VSLAM, perception, and navigation"
sidebar_label: "Hardware-Accelerated AI with Isaac ROS"
sidebar_position: 2
keywords: [Isaac ROS, hardware acceleration, VSLAM, perception, navigation, GPU, CUDA, robotics]
learning_outcomes:
  - Implement Isaac ROS pipelines that leverage NVIDIA GPU acceleration
  - Understand VSLAM implementation with hardware acceleration
  - Configure perception and navigation pipelines using Isaac ROS packages
  - Measure performance improvements from hardware acceleration
---

# Hardware-Accelerated AI with Isaac ROS

## Learning Objectives

After completing this chapter, you will be able to:
- Implement Isaac ROS pipelines that leverage NVIDIA GPU acceleration for real-time performance
- Understand and configure VSLAM (Visual Simultaneous Localization and Mapping) systems with hardware acceleration
- Configure perception and navigation pipelines using Isaac ROS packages
- Measure and validate performance improvements from hardware acceleration
- Integrate Isaac ROS with ROS 2 systems for comprehensive AI-driven robotics applications

## Table of Contents
- [Introduction to Isaac ROS](#introduction-to-isaac-ros)
- [Setting Up Isaac ROS Environment](#setting-up-isaac-ros-environment)
- [VSLAM with Hardware Acceleration](#vslam-with-hardware-acceleration)
- [Perception Pipelines](#perception-pipelines)
- [Navigation with Isaac ROS](#navigation-with-isaac-ros)
- [Performance Optimization](#performance-optimization)
- [Integration with ROS 2](#integration-with-ros-2)
- [Summary](#summary)

## Introduction to Isaac ROS

Isaac ROS is NVIDIA's collection of hardware-accelerated perception and navigation packages designed specifically for robotics applications. It provides optimized implementations of common robotics algorithms that leverage NVIDIA GPUs and specialized accelerators like Tensor Cores for maximum performance.

### Key Features of Isaac ROS
- **Hardware Acceleration**: Leverages CUDA, Tensor Cores, and other NVIDIA accelerators
- **Real-time Performance**: Optimized for real-time processing of sensor data
- **ROS 2 Integration**: Seamless integration with ROS 2 ecosystem
- **Modular Architecture**: Flexible pipeline construction with composable nodes
- **Specialized Algorithms**: Optimized implementations of SLAM, perception, and navigation algorithms

### Isaac ROS Package Categories

1. **Perception Packages**: Visual SLAM, object detection, segmentation
2. **Navigation Packages**: Path planning, obstacle avoidance, localization
3. **Utility Packages**: Image processing, point cloud operations, calibration
4. **Sensor Packages**: Specialized drivers and processors for various sensors

## Setting Up Isaac ROS Environment

### Prerequisites

Before implementing Isaac ROS pipelines, ensure your system meets the following requirements:

- NVIDIA GPU with Compute Capability 6.0 or higher (Pascal architecture or newer)
- NVIDIA GPU driver version 470 or higher
- CUDA Toolkit 11.8 or later
- cuDNN library
- ROS 2 Humble Hawksbill or Rolling Ridley
- Isaac ROS packages installed

### Installation Steps

1. **Install NVIDIA Container Toolkit** (recommended for containerized deployments):
```bash
# Add NVIDIA package repository
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add repository
echo "deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://nvidia.github.io/libnvidia-container/stable/ubuntu20.04/amd64 /" | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update and install
sudo apt-get update
sudo apt-get install nvidia-container-toolkit
```

2. **Install Isaac ROS Packages**:
```bash
# Add Isaac ROS repository
sudo apt update && sudo apt install wget gnupg
wget -O - https://repo.download.nvidia.com/gpgkey/nvidia.pub | sudo apt-key add -
echo 'deb https://repo.download.nvidia.com/ $(lsb_release -cs)/main' | sudo tee /etc/apt/sources.list.d/nvidia-isaac.list

# Install Isaac ROS packages
sudo apt update
sudo apt install nvidia-isaac-ros-dev-isolated
```

### Basic Isaac ROS Pipeline Structure

An Isaac ROS pipeline typically follows this structure:

```yaml
# Example Isaac ROS pipeline configuration
name: perception_pipeline
nodes:
  # Image preprocessing node
  - name: image_processor
    package: isaac_ros_image_proc
    executable: image_processor_node
    parameters:
      - config/image_processor.yaml

  # Feature extraction node
  - name: feature_detector
    package: isaac_ros_feature_detection
    executable: feature_detector_node
    parameters:
      - config/feature_detector.yaml

  # Hardware-accelerated processing node
  - name: cuda_processor
    package: isaac_ros_cuda_processor
    executable: cuda_processor_node
    parameters:
      - config/cuda_processor.yaml

  # Output publisher
  - name: result_publisher
    package: isaac_ros_result_publisher
    executable: result_publisher_node
    parameters:
      - config/result_publisher.yaml
```

## VSLAM with Hardware Acceleration

### Visual SLAM Fundamentals

Visual SLAM (Simultaneous Localization and Mapping) uses camera inputs to simultaneously build a map of the environment and localize the robot within it. Isaac ROS provides hardware-accelerated implementations that significantly improve performance.

### Isaac ROS Visual SLAM Pipeline

```python
# Example Isaac ROS Visual SLAM implementation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray

class IsaacVSLAMNode(Node):
    def __init__(self):
        super().__init__('isaac_vsiam_node')

        # Subscriptions for camera data
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        # Publishers for SLAM results
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/visual_slam/pose',
            10
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/visual_slam/odometry',
            10
        )

        # Initialize Isaac ROS VSLAM components
        self.initialize_vsiam_components()

        # Performance monitoring
        self.last_process_time = self.get_clock().now()

    def initialize_vsiam_components(self):
        """Initialize hardware-accelerated VSLAM components"""
        # This would typically use Isaac ROS packages like:
        # - Isaac ROS AprilTag 2D
        # - Isaac ROS AprilTag 3D
        # - Isaac ROS Stereo Dense Reconstruction
        # - Isaac ROS Visual Slam

        # Placeholder for actual Isaac ROS VSLAM initialization
        self.get_logger().info('Isaac ROS VSLAM components initialized')

    def image_callback(self, msg):
        """Process incoming image data with hardware acceleration"""
        start_time = self.get_clock().now()

        # Process image using Isaac ROS hardware-accelerated nodes
        # This would typically involve:
        # 1. Feature extraction using CUDA
        # 2. Descriptor matching on GPU
        # 3. Pose estimation with Tensor Cores
        processed_data = self.accelerated_image_processing(msg)

        # Update SLAM map and estimate pose
        pose_estimate = self.update_vsiam_map(processed_data)

        # Publish results
        self.publish_pose_estimate(pose_estimate)

        # Log performance metrics
        end_time = self.get_clock().now()
        processing_time = (end_time - start_time).nanoseconds / 1e6  # ms
        self.get_logger().info(f'VSLAM processing time: {processing_time:.2f} ms')

    def accelerated_image_processing(self, image_msg):
        """Hardware-accelerated image processing using Isaac ROS"""
        # This function would interface with Isaac ROS packages
        # that perform feature detection, descriptor extraction,
        # and matching using GPU acceleration

        # Example: Using Isaac ROS AprilTag 2D for feature detection
        # apriltag_detections = self.apriltag_detector.detect(image_msg)

        # Example: Using Isaac ROS Stereo Dense Reconstruction
        # depth_map = self.stereo_reconstructor.compute_depth(image_msg)

        # Return processed features/descriptors
        return {
            'features': [],
            'descriptors': [],
            'timestamp': image_msg.header.stamp
        }

    def update_vsiam_map(self, processed_data):
        """Update VSLAM map and estimate current pose"""
        # This would use Isaac ROS Visual SLAM packages
        # to update the map and estimate robot pose
        return {
            'position': [0.0, 0.0, 0.0],
            'orientation': [0.0, 0.0, 0.0, 1.0],  # quaternion
            'confidence': 0.95
        }

    def publish_pose_estimate(self, pose_estimate):
        """Publish pose estimate to ROS topics"""
        # Create and publish pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = pose_estimate['position'][0]
        pose_msg.pose.position.y = pose_estimate['position'][1]
        pose_msg.pose.position.z = pose_estimate['position'][2]

        pose_msg.pose.orientation.x = pose_estimate['orientation'][0]
        pose_msg.pose.orientation.y = pose_estimate['orientation'][1]
        pose_msg.pose.orientation.z = pose_estimate['orientation'][2]
        pose_msg.pose.orientation.w = pose_estimate['orientation'][3]

        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)

    vsiam_node = IsaacVSLAMNode()

    try:
        rclpy.spin(vsiam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vsiam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Performance Optimization for VSLAM

```yaml
# Isaac ROS VSLAM configuration for optimal performance
vsiam_config:
  # Feature detection parameters
  feature_detector:
    max_features: 1000
    quality_level: 0.01
    min_distance: 10.0
    block_size: 3

  # Hardware acceleration settings
  cuda_settings:
    use_tensor_cores: true
    memory_pool_size: 512  # MB
    stream_priority: 0     # Normal priority

  # Processing parameters
  processing:
    frame_skip: 1          # Process every frame
    max_keyframe_interval: 30  # Max frames between keyframes
    tracking_threshold: 10 # Min features for tracking

  # Map management
  map_management:
    max_map_size: 10000    # Max landmarks in map
    min_triangulation_angle: 0.1  # Radians
    outlier_rejection_threshold: 3.0  # Mahalanobis distance
```

## Perception Pipelines

### Isaac ROS Perception Components

Isaac ROS provides several hardware-accelerated perception components:

#### Object Detection Pipeline

```python
# Isaac ROS object detection pipeline
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import Header

class IsaacObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('isaac_object_detection_node')

        # Subscription to camera input
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publisher for detection results
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            '/isaac_ros/detections',
            10
        )

        # Initialize Isaac ROS object detection
        self.setup_object_detector()

    def setup_object_detector(self):
        """Initialize Isaac ROS hardware-accelerated object detector"""
        # This would typically use Isaac ROS Detection 2D package
        # which leverages TensorRT for inference acceleration
        self.get_logger().info('Isaac ROS object detector initialized')

    def image_callback(self, msg):
        """Process image and detect objects using hardware acceleration"""
        # Process image with Isaac ROS object detection
        detections = self.accelerated_object_detection(msg)

        # Create detection message
        detection_msg = Detection2DArray()
        detection_msg.header = msg.header
        detection_msg.detections = detections

        # Publish results
        self.detection_publisher.publish(detection_msg)

    def accelerated_object_detection(self, image_msg):
        """Hardware-accelerated object detection using Isaac ROS"""
        # This would interface with Isaac ROS packages like:
        # - Isaac ROS Detection 2D (TensorRT acceleration)
        # - Isaac ROS Detection 3D
        # - Isaac ROS Segmentation

        # Return list of Detection2D objects
        return []
```

#### Stereo Dense Reconstruction Pipeline

```python
# Isaac ROS stereo reconstruction pipeline
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import numpy as np

class IsaacStereoReconstructionNode(Node):
    def __init__(self):
        super().__init__('isaac_stereo_reconstruction_node')

        # Subscriptions for stereo camera pair
        self.left_subscription = self.create_subscription(
            Image,
            '/stereo/left/image_rect',
            self.left_image_callback,
            10
        )

        self.right_subscription = self.create_subscription(
            Image,
            '/stereo/right/image_rect',
            self.right_image_callback,
            10
        )

        # Camera info subscriptions
        self.left_info_subscription = self.create_subscription(
            CameraInfo,
            '/stereo/left/camera_info',
            self.left_camera_info_callback,
            10
        )

        self.right_info_subscription = self.create_subscription(
            CameraInfo,
            '/stereo/right/camera_info',
            self.right_camera_info_callback,
            10
        )

        # Publisher for dense point cloud
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            '/isaac_ros/pointcloud_dense',
            10
        )

        self.cv_bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.camera_params = {}

        # Initialize Isaac ROS stereo components
        self.initialize_stereo_components()

    def initialize_stereo_components(self):
        """Initialize Isaac ROS hardware-accelerated stereo components"""
        # This would use Isaac ROS Stereo Dense Reconstruction package
        # which leverages CUDA for disparity computation
        self.get_logger().info('Isaac ROS stereo reconstruction initialized')

    def left_image_callback(self, msg):
        """Process left camera image"""
        self.left_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_stereo_pair_if_ready()

    def right_image_callback(self, msg):
        """Process right camera image"""
        self.right_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_stereo_pair_if_ready()

    def process_stereo_pair_if_ready(self):
        """Process stereo pair if both images are available"""
        if self.left_image is not None and self.right_image is not None:
            # Perform hardware-accelerated stereo reconstruction
            pointcloud = self.accelerated_stereo_reconstruction(
                self.left_image,
                self.right_image
            )

            # Publish dense point cloud
            if pointcloud is not None:
                self.pointcloud_publisher.publish(pointcloud)

            # Clear images to save memory
            self.left_image = None
            self.right_image = None

    def accelerated_stereo_reconstruction(self, left_img, right_img):
        """Hardware-accelerated stereo reconstruction using Isaac ROS"""
        # This would interface with Isaac ROS Stereo Dense Reconstruction
        # package which uses CUDA for disparity map computation
        # and subsequent 3D point cloud generation

        # Placeholder for actual Isaac ROS stereo processing
        return None
```

## Navigation with Isaac ROS

### Isaac ROS Navigation Stack Integration

Isaac ROS enhances the traditional ROS 2 navigation stack with hardware acceleration:

```yaml
# Isaac ROS enhanced navigation configuration
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 10.0
      global_frame: odom
      robot_base_frame: base_link
      use_roll_pitch: true  # Use Isaac ROS enhanced 3D awareness
      resolution: 0.05

      # Isaac ROS enhanced plugins
      plugins: [
        "isaac_ros_obstacle_layer",
        "isaac_ros_static_layer",
        "isaac_ros_voxel_layer"
      ]

      # Isaac ROS obstacle layer configuration
      isaac_ros_obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /isaac_ros/processed_scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          queue_size: 10

global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[-0.325, -0.325], [-0.325, 0.325], [0.325, 0.325], [0.325, -0.325]]"
      robot_base_frame: base_link
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      resolution: 0.05

      # Isaac ROS enhanced plugins
      plugins: [
        "isaac_ros_static_layer",
        "isaac_ros_obstacle_layer"
      ]

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    # Use Isaac ROS enhanced planners
    planner_plugins: ["IsaacRRTPlanner"]

    IsaacRRTPlanner:
      plugin: "isaac_ros.nav2.planners.IsaacRRTPlanner"
      max_iterations: 1000
      step_size: 0.5
      goal_bias: 0.05

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop"]

    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"

    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    use_sim_time: true
```

### Isaac ROS Path Planning with Hardware Acceleration

```python
# Isaac ROS hardware-accelerated path planning
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
import numpy as np

class IsaacPathPlannerNode(Node):
    def __init__(self):
        super().__init__('isaac_path_planner_node')

        # Service for path planning requests
        self.path_plan_service = self.create_service(
            GetPlan,
            '/isaac_ros/planner/get_plan',
            self.handle_path_plan_request
        )

        # Publisher for planned paths
        self.path_publisher = self.create_publisher(
            Path,
            '/isaac_ros/local_plan',
            10
        )

        # Initialize Isaac ROS path planning components
        self.initialize_path_planner()

    def initialize_path_planner(self):
        """Initialize Isaac ROS hardware-accelerated path planner"""
        # This would use Isaac ROS enhanced planners that leverage:
        # - GPU-accelerated collision checking
        # - Parallel path optimization
        # - Tensor Core acceleration for neural planning networks
        self.get_logger().info('Isaac ROS path planner initialized')

    def handle_path_plan_request(self, request, response):
        """Handle path planning request with hardware acceleration"""
        start_time = self.get_clock().now()

        # Plan path using Isaac ROS hardware-accelerated planner
        path = self.accelerated_path_planning(
            request.start,
            request.goal,
            request.tolerance
        )

        # Calculate performance metrics
        end_time = self.get_clock().now()
        planning_time = (end_time - start_time).nanoseconds / 1e6  # ms

        response.plan = path
        self.get_logger().info(f'Path planning completed in {planning_time:.2f} ms')

        return response

    def accelerated_path_planning(self, start, goal, tolerance):
        """Hardware-accelerated path planning using Isaac ROS"""
        # This would interface with Isaac ROS enhanced planners
        # that use GPU acceleration for:
        # - Collision checking across multiple threads
        # - Parallel sampling in RRT-based planners
        # - Neural network acceleration for learned planning

        # Create a sample path (in real implementation, this would
        # use Isaac ROS path planning packages)
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        # Generate path points (simplified for example)
        num_points = 20
        start_pos = np.array([start.pose.position.x, start.pose.position.y])
        goal_pos = np.array([goal.pose.position.x, goal.pose.position.y])

        for i in range(num_points + 1):
            ratio = i / num_points
            pos = start_pos + ratio * (goal_pos - start_pos)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(pos[0])
            pose.pose.position.y = float(pos[1])
            pose.pose.position.z = 0.0
            # Add some orientation based on direction

            path.poses.append(pose)

        return path
```

## Performance Optimization

### Benchmarking Isaac ROS Performance

```python
# Isaac ROS performance benchmarking
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import psutil
import GPUtil

class IsaacPerformanceMonitorNode(Node):
    def __init__(self):
        super().__init__('isaac_performance_monitor')

        # Publishers for performance metrics
        self.cpu_usage_publisher = self.create_publisher(
            Float32,
            '/isaac_ros/performance/cpu_usage',
            10
        )

        self.gpu_usage_publisher = self.create_publisher(
            Float32,
            '/isaac_ros/performance/gpu_usage',
            10
        )

        self.memory_usage_publisher = self.create_publisher(
            Float32,
            '/isaac_ros/performance/memory_usage',
            10
        )

        # Timer for periodic monitoring
        self.monitor_timer = self.create_timer(
            1.0,  # 1 second interval
            self.monitor_performance
        )

    def monitor_performance(self):
        """Monitor and publish Isaac ROS performance metrics"""
        # CPU usage
        cpu_percent = psutil.cpu_percent(interval=1)

        # Memory usage
        memory_percent = psutil.virtual_memory().percent

        # GPU usage (if available)
        gpus = GPUtil.getGPUs()
        if gpus:
            gpu_percent = gpus[0].load * 100
        else:
            gpu_percent = 0.0

        # Publish metrics
        cpu_msg = Float32()
        cpu_msg.data = float(cpu_percent)
        self.cpu_usage_publisher.publish(cpu_msg)

        gpu_msg = Float32()
        gpu_msg.data = float(gpu_percent)
        self.gpu_usage_publisher.publish(gpu_msg)

        memory_msg = Float32()
        memory_msg.data = float(memory_percent)
        self.memory_usage_publisher.publish(memory_msg)

        # Log performance summary
        self.get_logger().info(
            f'Performance - CPU: {cpu_percent:.1f}%, '
            f'GPU: {gpu_percent:.1f}%, '
            f'Memory: {memory_percent:.1f}%'
        )
```

### Isaac ROS Configuration for Maximum Performance

```yaml
# Isaac ROS performance optimization configuration
performance_config:
  # CUDA memory management
  cuda_memory:
    pool_size: 1024  # MB
    enable_pool: true
    initial_pool_size: 512  # MB

  # Processing pipeline optimization
  pipeline:
    max_concurrent_processes: 4
    batch_size: 1
    enable_async_processing: true

  # Device selection
  device:
    compute_device: gpu  # or "cuda:0", "cuda:1", etc.
    fallback_to_cpu: true

  # Precision settings
  precision:
    float_precision: fp32  # or "fp16" for Tensor Core acceleration
    enable_tensor_cores: true

  # Resource limits
  resource_limits:
    max_gpu_memory_percentage: 80
    max_cpu_threads: 8
    max_queue_size: 100
```

## Integration with ROS 2

### Isaac ROS Bridge Configuration

```yaml
# Isaac ROS bridge configuration
bridge_config:
  # Sensor data bridging
  - ros_topic_name: "/isaac_ros/camera/image_raw"
    graph_resource_name: "isaac_ros_camera_publisher"
    type: "sensor_msgs/msg/Image"
    qos: "default"
    direction: "output"

  - ros_topic_name: "/isaac_ros/lidar/points"
    graph_resource_name: "isaac_ros_lidar_publisher"
    type: "sensor_msgs/msg/PointCloud2"
    qos: "default"
    direction: "output"

  # Control command bridging
  - ros_topic_name: "/isaac_ros/cmd_vel"
    graph_resource_name: "isaac_ros_cmd_vel_subscriber"
    type: "geometry_msgs/msg/Twist"
    qos: "default"
    direction: "input"

  # SLAM result bridging
  - ros_topic_name: "/isaac_ros/vslam/pose"
    graph_resource_name: "isaac_ros_pose_publisher"
    type: "geometry_msgs/msg/PoseStamped"
    qos: "default"
    direction: "output"

  - ros_topic_name: "/isaac_ros/vslam/map"
    graph_resource_name: "isaac_ros_map_publisher"
    type: "nav_msgs/msg/OccupancyGrid"
    qos: "default"
    direction: "output"
```

### Example Isaac ROS Application Node

```python
# Complete Isaac ROS application example
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import threading
import time

class IsaacRobotApplicationNode(Node):
    def __init__(self):
        super().__init__('isaac_robot_application')

        # Initialize Isaac ROS components
        self.initialize_isaac_ros_components()

        # Create subscribers for sensor data
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_callback, 10
        )

        # Create publishers for control and status
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/isaac_ros/status', 10)

        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10Hz

        # Status tracking
        self.is_operational = True
        self.last_sensor_data_time = self.get_clock().now()

    def initialize_isaac_ros_components(self):
        """Initialize all Isaac ROS accelerated components"""
        self.get_logger().info('Initializing Isaac ROS components...')

        # Initialize perception pipeline
        self.perception_pipeline = self.initialize_perception_pipeline()

        # Initialize VSLAM system
        self.vslam_system = self.initialize_vsiam_system()

        # Initialize navigation system
        self.navigation_system = self.initialize_navigation_system()

        self.get_logger().info('Isaac ROS components initialized successfully')

    def camera_callback(self, msg):
        """Process camera data through Isaac ROS perception pipeline"""
        # Process image using Isaac ROS accelerated perception
        perception_results = self.process_with_isaac_perception(msg)

        # Update VSLAM system with processed data
        self.update_vsiam_with_image(perception_results)

        self.last_sensor_data_time = self.get_clock().now()

    def lidar_callback(self, msg):
        """Process LiDAR data through Isaac ROS pipeline"""
        # Process LiDAR data using Isaac ROS accelerated processing
        processed_scan = self.process_with_isaac_lidar(msg)

        # Update navigation system with processed data
        self.update_navigation_with_scan(processed_scan)

        self.last_sensor_data_time = self.get_clock().now()

    def control_loop(self):
        """Main control loop using Isaac ROS outputs"""
        if not self.is_operational:
            return

        # Get current robot state from Isaac ROS systems
        robot_state = self.get_robot_state_from_isaac_ros()

        # Plan and execute navigation based on Isaac ROS perception
        navigation_command = self.plan_navigation(robot_state)

        # Publish command to robot
        self.cmd_pub.publish(navigation_command)

        # Publish operational status
        status_msg = Bool()
        status_msg.data = self.is_operational
        self.status_pub.publish(status_msg)

    def get_robot_state_from_isaac_ros(self):
        """Get current robot state from Isaac ROS systems"""
        # This would aggregate state from:
        # - VSLAM pose estimation
        # - Perception results
        # - Navigation system state
        return {
            'pose': self.get_current_pose(),
            'perceptions': self.get_current_perceptions(),
            'navigation_state': self.get_navigation_state()
        }

    def plan_navigation(self, robot_state):
        """Plan navigation using Isaac ROS accelerated systems"""
        # This would use Isaac ROS path planners and controllers
        # that leverage hardware acceleration
        cmd = Twist()
        cmd.linear.x = 0.5  # Default forward motion
        cmd.angular.z = 0.0  # No turning by default
        return cmd

def main(args=None):
    rclpy.init(args=args)

    app_node = IsaacRobotApplicationNode()

    try:
        rclpy.spin(app_node)
    except KeyboardInterrupt:
        app_node.get_logger().info('Shutting down Isaac ROS application...')
    finally:
        app_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

Isaac ROS provides powerful hardware acceleration capabilities that significantly enhance the performance of perception and navigation algorithms in robotics applications. By leveraging NVIDIA GPUs and specialized accelerators like Tensor Cores, Isaac ROS enables real-time processing of sensor data that would otherwise be computationally prohibitive on CPU-only systems.

The integration of Isaac ROS with standard ROS 2 systems provides a seamless transition from simulation to real-world deployment while maintaining the performance benefits of hardware acceleration. This makes it an ideal choice for implementing AI-driven robotics applications that require real-time perception and navigation capabilities.

## Next Steps

Continue with the next chapter in this module:

- [Path Planning with Nav2](./path-planning-nav2) - Learn how to adapt navigation systems for humanoid robot locomotion
- [Module 3 Summary](../module-3/) - Return to the module overview