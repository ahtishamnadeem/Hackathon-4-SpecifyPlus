---
title: "Python Agents and ROS 2 Control"
description: "Learn how to bridge Python AI agents to robot controllers using ROS 2"
sidebar_label: "Python Agents and ROS 2"
sidebar_position: 2
keywords: [ROS 2, Python, rclpy, agents, controllers, publishers, subscribers]
learning_outcomes:
  - Use rclpy to build ROS 2 nodes that interface with Python AI agents
  - Implement the bridge between Python AI agents and robot controllers
  - Create publishers, subscribers, services, and actions in ROS 2
  - Understand practical applications of Python agents in robotics
---

# Python Agents and ROS 2 Control

## Learning Objectives

After completing this chapter, you will be able to:
- Use rclpy to build ROS 2 nodes that interface with Python AI agents
- Implement the bridge between Python AI agents and robot controllers
- Create publishers, subscribers, services, and actions in ROS 2
- Understand practical applications of Python agents in robotics

## Table of Contents
- [Using rclpy to Build ROS 2 Nodes](#using-rclpy-to-build-ros2-nodes)
- [Bridging Python AI Agents to Robot Controllers](#bridging-python-ai-agents-to-robot-controllers)
- [Publishers, Subscribers, Services, and Actions](#publishers-subscribers-services-and-actions)
- [Practical Code Examples](#practical-code-examples)
- [Summary](#summary)

## Using rclpy to Build ROS 2 Nodes

`rclpy` is the Python client library for ROS 2, allowing Python programs to interface with the ROS 2 system. It provides the tools needed to create nodes, publish and subscribe to topics, and provide or use services.

### Basic Node Structure

```python
import rclpy
from rclpy.node import Node

class PythonAgentNode(Node):
    def __init__(self):
        super().__init__('python_agent_node')
        self.get_logger().info('Python Agent Node has been started')

def main(args=None):
    rclpy.init(args=args)
    node = PythonAgentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating Publishers and Subscribers

The most common communication pattern in ROS 2 involves publishers and subscribers:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AgentController(Node):
    def __init__(self):
        super().__init__('agent_controller')

        # Create a publisher
        self.publisher = self.create_publisher(String, 'agent_commands', 10)

        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.sensor_callback,
            10
        )

        # Timer for periodic publishing
        self.timer = self.create_timer(0.5, self.publish_command)
        self.i = 0

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor data: {msg.data}')
        # Process sensor data and make decisions
        self.process_sensor_data(msg.data)

    def publish_command(self):
        msg = String()
        msg.data = f'Command from agent: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

    def process_sensor_data(self, data):
        # Implement AI agent logic here
        self.get_logger().info(f'Agent processing: {data}')

def main(args=None):
    rclpy.init(args=args)
    agent_controller = AgentController()

    try:
        rclpy.spin(agent_controller)
    except KeyboardInterrupt:
        pass
    finally:
        agent_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Bridging Python AI Agents to Robot Controllers

One of the key applications of ROS 2 is bridging high-level AI agents with low-level robot controllers. This involves:

1. **Data Flow**: Sensor data flows from robot to AI agent
2. **Decision Making**: AI agent processes data and makes decisions
3. **Command Execution**: Commands flow from AI agent to robot controllers
4. **Feedback Loop**: Continuous loop for real-time control

### Example: Simple Navigation Agent

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class NavigationAgent(Node):
    def __init__(self):
        super().__init__('navigation_agent')

        # Subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.safe_distance = 1.0  # meters

    def scan_callback(self, msg):
        # Process laser scan data
        min_distance = min(msg.ranges)

        # Simple navigation logic
        cmd = Twist()

        if min_distance > self.safe_distance:
            # Move forward
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0
        else:
            # Stop and turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

        # Publish command to robot controller
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    nav_agent = NavigationAgent()

    try:
        rclpy.spin(nav_agent)
    except KeyboardInterrupt:
        pass
    finally:
        nav_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishers, Subscribers, Services, and Actions

### Publishers

Publishers send data to topics. They are asynchronous and follow a fire-and-forget pattern:

```python
# Creating a publisher
publisher = self.create_publisher(String, 'topic_name', queue_size)

# Publishing a message
msg = String()
msg.data = 'Hello from Python agent'
publisher.publish(msg)
```

### Subscribers

Subscribers receive data from topics:

```python
# Creating a subscriber
subscription = self.create_subscription(
    MessageType,
    'topic_name',
    callback_function,
    queue_size
)

# Callback function
def callback_function(self, msg):
    # Process received message
    self.get_logger().info(f'Received: {msg.data}')
```

### Services

Services provide synchronous request/response communication:

```python
# Creating a service server
self.srv = self.create_service(RequestType, 'service_name', self.service_callback)

def service_callback(self, request, response):
    # Process request and return response
    response.result = f'Processed: {request.input}'
    return response

# Using a service client
client = self.create_client(RequestType, 'service_name')
```

### Actions

Actions are used for long-running tasks with feedback:

```python
# Creating an action server
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci

self._action_server = ActionServer(
    self,
    Fibonacci,
    'fibonacci',
    self.execute_callback
)

def execute_callback(self, goal_handle):
    feedback_msg = Fibonacci.Feedback()
    feedback_msg.sequence = [0, 1]

    for i in range(1, goal_handle.request.order):
        feedback_msg.sequence.append(
            feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
        goal_handle.publish_feedback(feedback_msg)

    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = feedback_msg.sequence
    return result
```

## Practical Code Examples

### Example 1: AI Agent for Object Recognition

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class ObjectRecognitionAgent(Node):
    def __init__(self):
        super().__init__('object_recognition_agent')

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        self.result_pub = self.create_publisher(String, 'object_detection_result', 10)

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Perform object recognition (simplified example)
            result = self.recognize_objects(cv_image)

            # Publish result
            result_msg = String()
            result_msg.data = result
            self.result_pub.publish(result_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def recognize_objects(self, image):
        # Simplified object recognition logic
        # In practice, you would use a trained model like YOLO or similar
        height, width, _ = image.shape

        # Return a simple result for demonstration
        return f"Detected objects in {width}x{height} image"

def main(args=None):
    rclpy.init(args=args)
    agent = ObjectRecognitionAgent()

    try:
        rclpy.spin(agent)
    except KeyboardInterrupt:
        pass
    finally:
        agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter has covered how to bridge Python AI agents to robot controllers using ROS 2. You've learned to use rclpy to create nodes, implement publishers and subscribers, and work with services and actions. These concepts form the foundation for creating intelligent robotic systems that can interact with the physical world through ROS 2.

## Next Steps

Continue with the next chapter in this module:

- [Previous: Introduction to ROS 2](./intro-to-ros2.md) - Review fundamental ROS 2 concepts
- [Next: Humanoid Robot Modeling with URDF](./humanoid-urdf-modeling.md) - Learn how to model humanoid robots using URDF