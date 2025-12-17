---
title: "Path Planning with Nav2 for Humanoid Robots"
description: "Learn to adapt Nav2 for bipedal humanoid movement and navigation strategies"
sidebar_label: "Path Planning with Nav2 for Humanoid Robots"
sidebar_position: 3
keywords: [Nav2, path planning, navigation, humanoid, bipedal, ROS2, robotics]
learning_outcomes:
  - Configure Nav2 for humanoid robot navigation with bipedal locomotion constraints
  - Understand differences between wheeled and bipedal navigation approaches
  - Implement stability-aware path planning for humanoid robots
  - Adapt costmap and planner parameters for humanoid-specific requirements
---

# Path Planning with Nav2 for Humanoid Robots

## Learning Objectives

After completing this chapter, you will be able to:
- Configure Nav2 for humanoid robot navigation with bipedal locomotion constraints
- Understand the fundamental differences between wheeled and bipedal navigation approaches
- Implement stability-aware path planning for humanoid robots that maintains balance during navigation
- Adapt costmap and planner parameters for humanoid-specific requirements
- Integrate Nav2 with ROS 2 systems for comprehensive humanoid robot navigation

## Table of Contents
- [Introduction to Humanoid Navigation](#introduction-to-humanoid-navigation)
- [Differences from Wheeled Navigation](#differences-from-wheeled-navigation)
- [Humanoid-Specific Navigation Constraints](#humanoid-specific-navigation-constraints)
- [Adapting Nav2 for Bipedal Robots](#adapting-nav2-for-bipedal-robots)
- [Stability-Aware Path Planning](#stability-aware-path-planning)
- [Humanoid Navigation Strategies](#humanoid-navigation-strategies)
- [Integration with ROS 2 Systems](#integration-with-ros-2-systems)
- [Summary](#summary)

## Introduction to Humanoid Navigation

Navigation for humanoid robots presents unique challenges compared to traditional wheeled robots. While wheeled robots can move in any direction with simple differential or omni-directional drive systems, humanoid robots must maintain balance and stability while navigating. This requires considering the Center of Mass (CoM), Zero Moment Point (ZMP), and bipedal gait patterns during path planning and execution.

### Key Challenges in Humanoid Navigation

1. **Balance Maintenance**: Humanoid robots must maintain their center of mass within the support polygon defined by their feet
2. **Gait Planning**: Steps must be carefully planned to ensure stable locomotion
3. **Dynamic Stability**: Unlike wheeled robots, humanoid robots are dynamically unstable and require active control
4. **Footstep Planning**: Navigation paths must be translated into specific footstep sequences
5. **Terrain Adaptation**: Humanoid robots need to adapt their walking pattern to uneven terrain

### Humanoid vs. Wheeled Navigation Comparison

| Aspect | Wheeled Robot | Humanoid Robot |
|--------|---------------|----------------|
| Stability | Passive (always stable) | Active (requires balance control) |
| Mobility | Omnidirectional possible | Limited by bipedal constraints |
| Terrain | Smooth surfaces preferred | Can handle stairs, obstacles |
| Turning | Instantaneous rotation | Requires step-by-step repositioning |
| Stopping | Immediate | Requires deceleration and balance recovery |

## Differences from Wheeled Navigation

### Kinematic Constraints

Humanoid robots have significantly different kinematic constraints compared to wheeled robots:

- **Degrees of Freedom**: Higher complexity with multiple joints and limbs
- **Support Polygon**: Changes dynamically with foot placement
- **Step Size Limitations**: Finite maximum step length and height
- **Turning Radius**: Cannot turn in place like wheeled robots

### Dynamic Considerations

```yaml
# Example of humanoid-specific dynamic constraints
humanoid_constraints:
  # Balance constraints
  balance_margin: 0.1  # meters from edge of support polygon
  com_height_range: [0.7, 1.2]  # acceptable CoM height range
  zmp_deviation_limit: 0.05  # meters maximum ZMP deviation

  # Gait constraints
  max_step_length: 0.3  # meters
  max_step_width: 0.25  # meters
  max_step_height: 0.15  # meters (for stepping over obstacles)
  min_step_duration: 0.5  # seconds
  max_step_duration: 2.0  # seconds

  # Turning constraints
  max_turn_angle: 0.5  # radians per step
  turn_radius: 0.4  # minimum turning radius in meters
```

### Footstep Planning Requirements

Unlike wheeled robots that can be represented as a simple circular footprint, humanoid robots require footstep planning:

```python
# Example footstep planning structure
class Footstep:
    def __init__(self, position, orientation, step_type='normal'):
        self.position = position  # 3D position [x, y, z]
        self.orientation = orientation  # quaternion [x, y, z, w]
        self.step_type = step_type  # 'normal', 'stepping_stone', 'stairs_up', etc.
        self.support_polygon = self.calculate_support_polygon()  # convex hull of foot

    def calculate_support_polygon(self):
        # Calculate the support polygon based on foot geometry
        # This is critical for balance during navigation
        pass

class FootstepPlan:
    def __init__(self):
        self.footsteps = []  # List of Footstep objects
        self.support_polygons = []  # Corresponding support polygons
        self.balance_constraints = []  # Balance constraints for each step
```

## Humanoid-Specific Navigation Constraints

### Stability Constraints

Humanoid robots must satisfy stability constraints during navigation:

1. **Zero Moment Point (ZMP) Constraint**: The ZMP must remain within the support polygon
2. **Capture Point Constraint**: The robot's capture point must be within the next foot placement
3. **Centroidal Momentum Constraint**: Angular momentum around the CoM must be controlled

### Costmap Adaptations

Nav2's costmap system needs to be adapted for humanoid robots:

```yaml
# Humanoid-specific costmap configuration
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_roll_pitch: true  # Consider 3D orientation for humanoid
      use_height_for_cost: true  # Consider terrain height variations
      resolution: 0.05  # Finer resolution for precise footstep planning

      # Humanoid-specific plugins
      plugins: [
        "humanoid_static_layer",
        "humanoid_obstacle_layer",
        "humanoid_footprint_layer",
        "humanoid_terrain_layer"
      ]

      # Static layer for known obstacles
      humanoid_static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      # Obstacle layer adapted for humanoid height
      humanoid_obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /laser_scan
          max_obstacle_height: 1.8  # Humanoid eye level
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

      # Footprint layer for bipedal navigation
      humanoid_footprint_layer:
        plugin: "nav2_costmap_2d::FootprintLayer"
        enabled: true
        footprint_padding: 0.05  # Small padding for footstep accuracy
        robot_radius: 0.15  # Effective radius for humanoid

      # Terrain analysis layer for humanoid locomotion
      humanoid_terrain_layer:
        plugin: "nav2_costmap_2d::TerrainAnalysisLayer"
        enabled: true
        max_walkable_slope: 0.3  # Maximum walkable slope (radians)
        min_navigable_area: 0.25  # Minimum area for stable footing (sq meters)
        obstacle_sensitivity: 0.8  # Sensitivity to terrain irregularities
```

### Planner Constraints

The path planners in Nav2 need to account for humanoid-specific constraints:

```yaml
# Humanoid-specific planner configuration
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["humanoid_rpp_planner"]

    # Humanoid-specific RPP (Recovery Planner) with balance constraints
    humanoid_rpp_planner:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      # Humanoid-specific parameters
      step_size_constraint: 0.3  # Maximum step size in meters
      turning_constraint: 0.5    # Maximum turn per step in radians
      terrain_constraint: 0.3    # Maximum slope constraint in radians
      stability_margin: 0.1      # Stability margin around obstacles
```

## Adapting Nav2 for Bipedal Robots

### Humanoid Navigation Stack Configuration

```yaml
# Complete humanoid navigation stack configuration
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_to_pose_bt_xml: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/humanoid_nav_to_pose_w_replanning_and_recovery.xml
    default_nav_through_poses_bt_xml: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/humanoid_nav_through_poses_w_replanning_and_recovery.xml

    # Behavior tree customization for humanoid navigation
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_compute_path_through_poses_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_assisted_teleop_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_truncate_path_action_bt_node
      - nav2_truncate_path_local_action_bt_node
      - nav2_goal_updater_node_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_path_expiring_timer_condition
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
      - nav2_is_battery_low_condition_bt_node
      - nav2_navigate_through_poses_action_bt_node
      - nav2_navigate_to_pose_action_bt_node
      - nav2_remove_passed_goals_action_bt_node
      - nav2_planner_selector_bt_node
      - nav2_controller_selector_bt_node
      - nav2_goal_checker_selector_bt_node
      - nav2_controller_cancel_bt_node
      - nav2_path_longer_on_approach_bt_node
      - nav2_wait_cancel_bt_node
      - humaniod_footstep_planner_bt_node  # Custom humanoid footstep planner
```

### Custom Humanoid Footstep Planner

```python
# Custom footstep planner for humanoid robots
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
from scipy.spatial import KDTree
import math

class HumanoidFootstepPlannerNode(Node):
    def __init__(self):
        super().__init__('humanoid_footstep_planner')

        # Publisher for footstep plans
        self.footstep_publisher = self.create_publisher(
            Path,
            '/humanoid/footstep_plan',
            10
        )

        # Service for computing footstep plans
        self.footstep_service = self.create_service(
            ComputePathToPose,
            '/humanoid/compute_footstep_plan',
            self.compute_footstep_plan
        )

        # Parameters for humanoid navigation
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_step_length', 0.3),
                ('max_step_width', 0.25),
                ('max_step_height', 0.15),
                ('min_step_spacing', 0.1),
                ('support_polygon_margin', 0.05),
                ('balance_threshold', 0.1)
            ]
        )

    def compute_footstep_plan(self, request, response):
        """Compute footstep plan for humanoid navigation"""
        start_pose = request.start
        goal_pose = request.goal
        tolerance = request.tolerance

        # Compute initial path using standard Nav2 planner
        initial_path = self.compute_initial_path(start_pose, goal_pose)

        # Convert path to footstep plan considering humanoid constraints
        footstep_plan = self.convert_path_to_footsteps(initial_path)

        # Validate and optimize footstep plan for stability
        validated_plan = self.validate_footstep_plan(footstep_plan)

        response.plan = validated_plan
        return response

    def convert_path_to_footsteps(self, path):
        """Convert a continuous path to discrete footsteps for humanoid"""
        footsteps = []

        # Start with current robot position
        current_left_foot = self.calculate_foot_position(path.poses[0], 'left')
        current_right_foot = self.calculate_foot_position(path.poses[0], 'right')

        # Alternate steps between left and right foot
        step_side = 'left'  # Start with left foot

        for i in range(len(path.poses)):
            pose = path.poses[i]

            # Calculate next foot position based on humanoid kinematics
            if step_side == 'left':
                next_foot = self.calculate_next_foot_position(
                    current_left_foot, current_right_foot, pose, 'left'
                )
                current_left_foot = next_foot
                step_side = 'right'
            else:
                next_foot = self.calculate_next_foot_position(
                    current_right_foot, current_left_foot, pose, 'right'
                )
                current_right_foot = next_foot
                step_side = 'left'

            # Ensure step constraints are satisfied
            if self.is_valid_footstep(next_foot, current_left_foot, current_right_foot):
                footsteps.append(next_foot)

        return self.create_path_from_footsteps(footsteps)

    def calculate_foot_position(self, robot_pose, foot_type):
        """Calculate initial foot position based on robot pose"""
        # For humanoid, feet are offset from robot center
        offset_x = 0.0  # No forward offset for centered stance
        offset_y = 0.1 if foot_type == 'left' else -0.1  # Lateral offset

        foot_pose = PoseStamped()
        foot_pose.header = robot_pose.header

        # Apply lateral offset to robot position
        foot_pose.pose.position.x = robot_pose.pose.position.x + offset_x
        foot_pose.pose.position.y = robot_pose.pose.position.y + offset_y
        foot_pose.pose.position.z = robot_pose.pose.position.z  # Ground level

        # Orientation matches robot orientation
        foot_pose.pose.orientation = robot_pose.pose.orientation

        return foot_pose

    def calculate_next_foot_position(self, current_foot, other_foot, target_pose, foot_type):
        """Calculate next foot position considering balance and step constraints"""
        # Calculate target position based on path and step constraints
        dx = target_pose.pose.position.x - current_foot.pose.position.x
        dy = target_pose.pose.position.y - current_foot.pose.position.y

        distance = math.sqrt(dx*dx + dy*dy)

        # Limit step size to humanoid constraints
        max_step = self.get_parameter('max_step_length').value
        if distance > max_step:
            dx = dx * max_step / distance
            dy = dy * max_step / distance

        # Calculate new foot position
        new_pose = PoseStamped()
        new_pose.header = target_pose.header
        new_pose.pose.position.x = current_foot.pose.position.x + dx
        new_pose.pose.position.y = current_foot.pose.position.y + dy
        new_pose.pose.position.z = current_foot.pose.position.z  # Maintain ground level

        # Preserve orientation
        new_pose.pose.orientation = target_pose.pose.orientation

        return new_pose

    def is_valid_footstep(self, foot_pose, left_foot, right_foot):
        """Check if footstep maintains humanoid balance"""
        # Calculate support polygon from current foot positions
        support_polygon = self.calculate_support_polygon(left_foot, right_foot)

        # Check if new foot position is within balance constraints
        balance_margin = self.get_parameter('balance_threshold').value

        # Verify new foot position maintains stability
        is_stable = self.check_balance_constraint(foot_pose, support_polygon, balance_margin)

        return is_stable

    def check_balance_constraint(self, foot_pose, support_polygon, margin):
        """Check if foot placement satisfies balance constraints"""
        # For simplicity, check if foot is within reasonable distance
        # from other foot to maintain bipedal stability
        # In real implementation, this would check ZMP and CoM constraints

        return True  # Placeholder - implement proper balance checking

    def validate_footstep_plan(self, footstep_plan):
        """Validate and optimize footstep plan for humanoid navigation"""
        # Check each step for validity and optimize path
        validated_poses = []

        for pose in footstep_plan.poses:
            # Additional validation could include:
            # - Terrain analysis for step stability
            # - Obstacle clearance for foot placement
            # - Slope constraints for step placement
            validated_poses.append(pose)

        footstep_plan.poses = validated_poses
        return footstep_plan
```

## Stability-Aware Path Planning

### Center of Mass (CoM) Trajectory Planning

For stable humanoid navigation, the path planner must consider the robot's center of mass trajectory:

```python
# CoM trajectory planning for stable navigation
class ComTrajectoryPlanner:
    def __init__(self, robot_mass, robot_height):
        self.robot_mass = robot_mass
        self.robot_height = robot_height
        self.gravity = 9.81  # m/s^2

    def plan_com_trajectory(self, footstep_plan, gait_pattern='walking'):
        """Plan CoM trajectory for given footstep plan"""
        com_trajectory = []

        for i, footstep in enumerate(footstep_plan):
            # Calculate CoM position based on support polygon
            if gait_pattern == 'walking':
                com_position = self.calculate_walking_com(footstep, i)
            elif gait_pattern == 'trotting':
                com_position = self.calculate_trotting_com(footstep, i)
            else:
                com_position = self.calculate_standing_com(footstep)

            com_trajectory.append(com_position)

        return com_trajectory

    def calculate_walking_com(self, footstep, step_index):
        """Calculate CoM position for walking gait"""
        # Walking gait CoM calculation
        # CoM should follow the moving support polygon
        com_x = footstep.pose.position.x + 0.1  # Slightly ahead of foot
        com_y = footstep.pose.position.y + 0.05  # Slightly toward stance foot
        com_z = self.robot_height * 0.55  # 55% of robot height

        return Point(x=com_x, y=com_y, z=com_z)

    def check_dynamic_stability(self, com_trajectory, footstep_plan):
        """Check if CoM trajectory maintains dynamic stability"""
        for i, com_pos in enumerate(com_trajectory):
            # Calculate ZMP (Zero Moment Point)
            zmp_x = com_pos.x - (com_pos.z / self.gravity) * self.calculate_com_acceleration_x(i, com_trajectory)
            zmp_y = com_pos.y - (com_pos.z / self.gravity) * self.calculate_com_acceleration_y(i, com_trajectory)

            # Check if ZMP is within support polygon
            if not self.is_zmp_stable(zmp_x, zmp_y, footstep_plan[i]):
                return False, f"ZMP unstable at step {i}: ({zmp_x:.3f}, {zmp_y:.3f})"

        return True, "Trajectory is dynamically stable"
```

### Humanoid-Specific Recovery Behaviors

```yaml
# Humanoid-specific recovery behaviors
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["humanoid_spin", "humanoid_backup", "humanoid_stabilize"]
    humanoid_spin:
      plugin: "nav2_recoveries/Spin"
      enabled: True
      frequency: 5.0
      sim_frequency: 50.0
      # Humanoid-specific spin parameters
      max_rotational_vel: 0.3  # Reduced for stability
      min_rotational_vel: 0.1
      rotational_acc_lim: 0.2
      # Balance considerations
      balance_check_enabled: true
      balance_threshold: 0.15  # meters from support polygon edge

    humanoid_backup:
      plugin: "nav2_recoveries/BackUp"
      enabled: True
      frequency: 5.0
      sim_frequency: 50.0
      # Humanoid-specific backup parameters
      backup_dist: -0.2  # Backup distance in meters
      backup_speed: 0.05  # Slow backup speed for stability
      # Balance considerations
      step_back_enabled: true  # Use footstep-based backing up
      max_steps_back: 3      # Maximum number of steps to back up

    humanoid_stabilize:
      plugin: "nav2_recoveries/Stabilize"
      enabled: True
      frequency: 10.0
      sim_frequency: 100.0
      # Humanoid-specific stabilization
      time_allowance: 5.0  # Time to achieve stability
      velocity_threshold: 0.01  # Velocity threshold for stability
      # Balance control parameters
      com_control_enabled: true
      zmp_control_enabled: true
      max_com_adjustment: 0.05  # Maximum CoM adjustment (meters)
```

## Humanoid Navigation Strategies

### Multi-Layer Navigation Architecture

Humanoid navigation typically requires a multi-layer approach:

```python
# Multi-layer humanoid navigation architecture
class HumanoidNavigationManager(Node):
    def __init__(self):
        super().__init__('humanoid_navigation_manager')

        # Global path planner (ignores humanoid-specific constraints initially)
        self.global_planner = GlobalPathPlanner()

        # Footstep planner (converts global path to footsteps)
        self.footstep_planner = HumanoidFootstepPlannerNode()

        # Balance controller (maintains stability during navigation)
        self.balance_controller = BalanceController()

        # Gait controller (executes footstep plan with proper gait)
        self.gait_controller = GaitController()

        # Publishers for different layers
        self.global_path_pub = self.create_publisher(Path, '/humanoid/global_path', 10)
        self.footstep_plan_pub = self.create_publisher(Path, '/humanoid/footstep_plan', 10)
        self.balance_status_pub = self.create_publisher(BalanceStatus, '/humanoid/balance_status', 10)

    def execute_navigation(self, goal_pose):
        """Execute complete humanoid navigation with all layers"""
        # 1. Compute global path
        global_path = self.global_planner.compute_path(goal_pose)
        self.global_path_publisher.publish(global_path)

        # 2. Convert to footstep plan
        footstep_plan = self.footstep_planner.convert_path_to_footsteps(global_path)
        self.footstep_plan_publisher.publish(footstep_plan)

        # 3. Execute with balance control
        success = self.execute_with_balance_control(footstep_plan)

        return success

    def execute_with_balance_control(self, footstep_plan):
        """Execute footstep plan with active balance control"""
        for i, footstep in enumerate(footstep_plan.poses):
            # Check balance before taking step
            if not self.balance_controller.is_balanced():
                self.get_logger().warn(f'Robot not balanced at step {i}, attempting recovery')

                # Attempt balance recovery
                recovery_success = self.balance_controller.attempt_recovery()
                if not recovery_success:
                    return False

            # Execute footstep with gait controller
            step_success = self.gait_controller.execute_footstep(footstep)

            if not step_success:
                self.get_logger().error(f'Failed to execute footstep {i}')
                return False

            # Update balance controller with new foot position
            self.balance_controller.update_support_polygon(footstep)

        return True
```

### Adaptive Navigation for Different Terrains

```python
# Terrain-adaptive navigation for humanoid robots
class TerrainAdaptiveNavigator:
    def __init__(self):
        self.terrain_classifier = TerrainClassifier()
        self.gait_selector = GaitSelector()
        self.step_adjuster = StepAdjuster()

    def adapt_navigation_for_terrain(self, current_terrain_type):
        """Adapt navigation parameters based on terrain type"""
        if current_terrain_type == 'flat_ground':
            return self.configure_flat_ground_navigation()
        elif current_terrain_type == 'uneven_ground':
            return self.configure_uneven_ground_navigation()
        elif current_terrain_type == 'stairs':
            return self.configure_stairs_navigation()
        elif current_terrain_type == 'narrow_passage':
            return self.configure_narrow_passage_navigation()
        elif current_terrain_type == 'obstacle_field':
            return self.configure_obstacle_navigation()
        else:
            return self.configure_default_navigation()

    def configure_flat_ground_navigation(self):
        """Optimize for flat, stable ground"""
        config = {
            'max_step_length': 0.3,
            'max_step_width': 0.25,
            'step_height': 0.05,
            'step_timing': 0.8,
            'balance_margin': 0.1,
            'gait_pattern': 'normal_walk'
        }
        return config

    def configure_uneven_ground_navigation(self):
        """Adapt for rough, uneven terrain"""
        config = {
            'max_step_length': 0.2,  # Shorter steps for stability
            'max_step_width': 0.2,   # Wider stance for balance
            'step_height': 0.1,      # Allow for height variations
            'step_timing': 1.2,      # Slower for careful placement
            'balance_margin': 0.15,  # Extra stability margin
            'gait_pattern': 'careful_walk'
        }
        return config

    def configure_stairs_navigation(self):
        """Configure for stair climbing/descending"""
        config = {
            'max_step_length': 0.25,
            'max_step_width': 0.25,
            'step_height': 0.18,     # Standard stair height
            'step_timing': 1.5,      # Extra time for stair navigation
            'balance_margin': 0.12,
            'gait_pattern': 'stairs_walk',
            'step_type': 'stairs_up'  # or 'stairs_down'
        }
        return config
```

## Integration with ROS 2 Systems

### Nav2 Lifecycle Management for Humanoid Robots

```python
# Lifecycle management for humanoid Nav2 system
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

class HumanoidNavLifecycleManager:
    def __init__(self):
        self.lifecycle_nodes = [
            '/local_costmap/local_costmap',
            '/global_costmap/global_costmap',
            '/planner_server',
            '/controller_server',
            '/humanoid_footstep_planner',
            '/balance_controller'
        ]

    def activate_humanoid_navigation(self):
        """Activate all humanoid navigation components"""
        for node_name in self.lifecycle_nodes:
            self.activate_lifecycle_node(node_name)

    def deactivate_humanoid_navigation(self):
        """Deactivate all humanoid navigation components"""
        for node_name in reversed(self.lifecycle_nodes):
            self.deactivate_lifecycle_node(node_name)

    def activate_lifecycle_node(self, node_name):
        """Activate a lifecycle node"""
        # Create client for lifecycle change
        client = self.create_client(ChangeState, f'{node_name}/change_state')

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for lifecycle service for {node_name}')

        # Activate the node
        request = ChangeState.Request()
        request.transition.id = Transition.TRANSITION_ACTIVATE
        request.transition.label = 'activate'

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Successfully activated {node_name}')
        else:
            self.get_logger().error(f'Failed to activate {node_name}')
```

### Example Launch File for Humanoid Navigation

```python
# Launch file for complete humanoid navigation system
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    # Navigation server
    navigation_server_cmd = Node(
        package='nav2_behavior_tree',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=nav_remappings
    )

    # Humanoid footstep planner
    footstep_planner_cmd = Node(
        package='humanoid_nav2_plugins',
        executable='humanoid_footstep_planner',
        name='humanoid_footstep_planner',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=nav_remappings
    )

    # Balance controller
    balance_controller_cmd = Node(
        package='humanoid_balance_control',
        executable='balance_controller',
        name='balance_controller',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=nav_remappings
    )

    # Lifecyle manager
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['controller_server',
                                   'planner_server',
                                   'recoveries_server',
                                   'bt_navigator',
                                   'waypoint_follower',
                                   'humanoid_footstep_planner',
                                   'balance_controller']}]
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('humanoid_nav2_config'), 'params', 'humanoid_nav2_params.yaml']),
            description='Full path to the ROS2 parameters file to use'),

        # Launch nodes
        lifecycle_manager_cmd,
        navigation_server_cmd,
        footstep_planner_cmd,
        balance_controller_cmd,
    ])
```

## Performance Considerations

### Real-Time Navigation Requirements

Humanoid robots have specific real-time requirements for navigation:

- **Control Frequency**: Typically 100-500 Hz for balance control
- **Path Planning Frequency**: 1-10 Hz depending on environment complexity
- **Footstep Planning**: Should complete within 100-200 ms for responsive navigation
- **Sensor Processing**: Must complete within sensor update cycle (typically < 50ms)

### Computational Complexity Management

```python
# Performance optimization for humanoid navigation
class PerformanceOptimizer:
    def __init__(self):
        self.max_planning_time = 0.1  # seconds
        self.planning_frequency = 5.0  # Hz
        self.use_approximation = True  # Use approximations for faster planning

    def optimize_footstep_planning(self, path_length, terrain_complexity):
        """Optimize footstep planning based on requirements"""
        if terrain_complexity > 0.8:  # Very complex terrain
            # Use more detailed planning with shorter horizons
            return {
                'horizon_steps': 10,
                'detailed_validation': True,
                'approximation_level': 'low'
            }
        elif path_length > 100:  # Long paths
            # Use approximation for distant steps
            return {
                'horizon_steps': 20,
                'detailed_validation': False,
                'approximation_level': 'high'
            }
        else:  # Default case
            return {
                'horizon_steps': 15,
                'detailed_validation': True,
                'approximation_level': 'medium'
            }
```

## Summary

Navigation for humanoid robots requires significant adaptations to the standard Nav2 framework to account for bipedal locomotion constraints, balance requirements, and footstep planning needs. The key differences include:

1. **Footstep Planning**: Converting continuous paths to discrete foot placements
2. **Balance Constraints**: Ensuring Center of Mass remains within support polygon
3. **Gait Adaptation**: Adjusting walking patterns based on terrain and obstacles
4. **Stability Control**: Maintaining dynamic stability during navigation
5. **Multi-layer Architecture**: Coordinating global planning, footstep generation, and balance control

The implementation of these concepts in Nav2 requires custom plugins, modified costmap layers, and specialized behavior trees that understand humanoid-specific constraints and capabilities.

## Next Steps

Continue with the module overview:

- [Module 3 Overview](./index) - Return to the module introduction
- [Physics Simulation with Isaac Sim](./physics-simulation-isaac-sim) - Review simulation concepts
- [Hardware-Accelerated AI with Isaac ROS](./hardware-accelerated-ai-isaac-ros) - Review perception systems