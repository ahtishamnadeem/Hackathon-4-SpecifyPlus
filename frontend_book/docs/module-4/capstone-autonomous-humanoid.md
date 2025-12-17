# Capstone Project: The Autonomous Humanoid

This capstone project integrates all concepts from Module 4 to create a complete autonomous humanoid system that responds to voice commands and executes complex robot behaviors.

## Overview

In this capstone project, you will combine voice processing, cognitive planning, and action execution to create an autonomous humanoid robot system. This project brings together all the concepts learned in this module into a cohesive, functional system.

## Learning Objectives

By the end of this capstone project, you will be able to:

- Integrate voice processing, cognitive planning, and action execution systems
- Create a complete workflow from voice command to robot action
- Implement error handling and recovery across the entire system
- Test and validate the autonomous humanoid system
- Debug and troubleshoot integrated robotic systems

## Project Architecture

The autonomous humanoid system consists of three main components that work together:

```
Voice Commands → [Voice Processing] → [Cognitive Planning] → [Action Execution] → Robot Actions
                     ↓                    ↓                      ↓
                Whisper API          LLM API               ROS 2 Actions
```

### Component Integration

Each component feeds into the next, creating a complete pipeline:

1. **Voice Processing Layer**: Converts speech to text and extracts command intent
2. **Cognitive Planning Layer**: Translates high-level commands into action sequences
3. **Action Execution Layer**: Executes ROS 2 actions on the humanoid robot

## Implementation Plan

### Step 1: System Architecture

First, let's design the main orchestrator that will coordinate all components:

```python
import threading
import queue
import time
from typing import Dict, Any, Callable
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class AutonomousHumanoidOrchestrator(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_orchestrator')

        # Initialize components
        self.voice_processor = VoiceCommandProcessor()
        self.cognitive_planner = ContextAwarePlanner(api_key="your-api-key")
        self.action_executor = ActionExecutor()

        # Communication queues
        self.voice_queue = queue.Queue()
        self.planning_queue = queue.Queue()
        self.execution_queue = queue.Queue()

        # Publishers and subscribers
        self.status_publisher = self.create_publisher(String, 'system_status', 10)
        self.voice_subscriber = self.create_subscription(
            String, 'voice_commands', self.voice_callback, 10
        )

        # Start processing threads
        self.voice_thread = threading.Thread(target=self.process_voice_commands)
        self.planning_thread = threading.Thread(target=self.process_planning)
        self.execution_thread = threading.Thread(target=self.process_execution)

        self.voice_thread.start()
        self.planning_thread.start()
        self.execution_thread.start()

        self.get_logger().info('Autonomous humanoid orchestrator initialized')

    def voice_callback(self, msg):
        """Handle incoming voice commands"""
        self.voice_queue.put(msg.data)
        self.publish_status("Received voice command, processing...")

    def process_voice_commands(self):
        """Process voice commands in a separate thread"""
        while rclpy.ok():
            try:
                command_text = self.voice_queue.get(timeout=1.0)

                # Process the voice command
                result = self.voice_processor.process_command_with_confidence(
                    command_text, self.voice_processor.model
                )

                if result['confidence'] > 0.7:
                    # Pass to planning stage
                    self.planning_queue.put({
                        'command': result['text'],
                        'original_command': command_text
                    })
                    self.publish_status(f"Command processed: {result['text']}")
                else:
                    # Handle unclear command
                    self.handle_unclear_command(result)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in voice processing: {e}")

    def process_planning(self):
        """Process planning in a separate thread"""
        while rclpy.ok():
            try:
                task = self.planning_queue.get(timeout=1.0)

                # Generate action sequence using LLM
                action_sequence = self.cognitive_planner.generate_contextual_action_sequence(
                    task['command']
                )

                if action_sequence:
                    # Pass to execution stage
                    self.execution_queue.put({
                        'action_sequence': action_sequence,
                        'original_command': task['original_command']
                    })
                    self.publish_status(f"Plan generated with {len(action_sequence)} actions")
                else:
                    self.publish_status("Failed to generate plan for command")

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in planning: {e}")

    def process_execution(self):
        """Process action execution in a separate thread"""
        while rclpy.ok():
            try:
                task = self.execution_queue.get(timeout=1.0)

                # Execute the action sequence
                success = self.action_executor.execute_action_sequence(
                    task['action_sequence']
                )

                if success:
                    self.publish_status("Command executed successfully")
                else:
                    self.publish_status("Command execution failed")

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Error in execution: {e}")

    def handle_unclear_command(self, result):
        """Handle commands with low confidence"""
        alternatives = result.get('alternatives', [])

        if alternatives:
            # Ask for clarification
            clarification_text = f"I heard: '{result['text']}', did you mean: {' or '.join(alternatives[:3])}?"
            self.speak(clarification_text)
        else:
            # Ask for repetition
            self.speak("I didn't understand that command. Please repeat.")

    def publish_status(self, status: str):
        """Publish system status"""
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def speak(self, text: str):
        """Make the robot speak"""
        # Implementation for text-to-speech
        self.get_logger().info(f"Robot says: {text}")
```

### Step 2: Action Executor

The action executor handles the actual execution of ROS 2 actions:

```python
class ActionExecutor:
    def __init__(self):
        # Initialize ROS 2 publishers for different actions
        self.cmd_vel_publisher = None  # Will be set by the orchestrator
        self.joint_publisher = None    # For arm movements
        self.tts_publisher = None      # For text-to-speech

        self.robot_state = {
            'position': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
            'battery_level': 100,
            'arm_position': 'neutral',
            'gripper_status': 'open'
        }

    def execute_action_sequence(self, action_sequence: list) -> bool:
        """Execute a sequence of actions"""
        for action in action_sequence:
            success = self.execute_single_action(action)

            if not success:
                self.get_logger().error(f"Action failed: {action}")
                return False

            # Small delay between actions to allow for completion
            time.sleep(0.2)

        return True

    def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """Execute a single action"""
        action_type = action.get('action', '')
        parameters = action.get('parameters', {})

        try:
            if action_type == 'move_forward':
                return self.move_forward(parameters.get('distance', 1.0))
            elif action_type == 'move_backward':
                return self.move_backward(parameters.get('distance', 1.0))
            elif action_type == 'turn_left':
                return self.turn_left(parameters.get('angle', 90.0))
            elif action_type == 'turn_right':
                return self.turn_right(parameters.get('angle', 90.0))
            elif action_type == 'raise_arm':
                return self.raise_arm(parameters.get('arm', 'right'))
            elif action_type == 'lower_arm':
                return self.lower_arm(parameters.get('arm', 'right'))
            elif action_type == 'speak':
                return self.speak(parameters.get('text', ''))
            elif action_type == 'navigate_to_location':
                return self.navigate_to_location(
                    parameters.get('x', 0.0),
                    parameters.get('y', 0.0)
                )
            elif action_type == 'detect_object':
                return self.detect_object(parameters.get('object_type', 'any'))
            elif action_type == 'grasp_object':
                return self.grasp_object()
            elif action_type == 'release_object':
                return self.release_object()
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing action {action_type}: {e}')
            return False

    def move_forward(self, distance: float) -> bool:
        """Move the robot forward by the specified distance"""
        # Create Twist message for forward movement
        twist_msg = Twist()
        twist_msg.linear.x = 0.5  # Speed in m/s
        twist_msg.angular.z = 0.0

        # Publish for the required duration
        duration = distance / 0.5
        start_time = time.time()

        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist_msg)
            time.sleep(0.01)

        # Stop the robot
        twist_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

        # Update robot state
        self.robot_state['position']['x'] += distance * 0.95  # Account for wheel slip

        return True

    def turn_left(self, angle: float) -> bool:
        """Turn the robot left by the specified angle in degrees"""
        # Convert to radians
        angle_rad = angle * 3.14159 / 180.0

        # Create Twist message for rotation
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.5  # Angular speed in rad/s

        # Calculate duration for the turn
        duration = angle_rad / 0.5
        start_time = time.time()

        while time.time() - start_time < duration:
            self.cmd_vel_publisher.publish(twist_msg)
            time.sleep(0.01)

        # Stop the robot
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

        # Update robot state
        self.robot_state['position']['theta'] += angle_rad

        return True

    def speak(self, text: str) -> bool:
        """Make the robot speak the specified text"""
        if self.tts_publisher:
            msg = String()
            msg.data = text
            self.tts_publisher.publish(msg)
            self.get_logger().info(f"Speaking: {text}")
            return True
        return False

    def navigate_to_location(self, x: float, y: float) -> bool:
        """Navigate to a specific location using path planning"""
        # This would typically use a navigation stack like Nav2
        # For simplicity, we'll implement basic navigation

        current_x = self.robot_state['position']['x']
        current_y = self.robot_state['position']['y']

        # Calculate direction and distance
        dx = x - current_x
        dy = y - current_y
        distance = (dx**2 + dy**2)**0.5

        if distance < 0.1:  # Already at destination
            return True

        # Calculate angle to target
        target_angle = math.atan2(dy, dx)
        angle_diff = target_angle - self.robot_state['position']['theta']

        # Normalize angle difference
        while angle_diff > 3.14159:
            angle_diff -= 2 * 3.14159
        while angle_diff < -3.14159:
            angle_diff += 2 * 3.14159

        # Turn to face the target
        if abs(angle_diff) > 0.1:
            self.turn_to_angle(target_angle)

        # Move forward to the target
        self.move_forward(distance)

        # Update position
        self.robot_state['position']['x'] = x
        self.robot_state['position']['y'] = y

        return True

    def turn_to_angle(self, target_angle: float) -> bool:
        """Turn the robot to face a specific angle"""
        current_angle = self.robot_state['position']['theta']
        angle_diff = target_angle - current_angle

        # Normalize angle difference
        while angle_diff > 3.14159:
            angle_diff -= 2 * 3.14159
        while angle_diff < -3.14159:
            angle_diff += 2 * 3.14159

        if angle_diff > 0:
            return self.turn_left(abs(angle_diff * 180 / 3.14159))
        else:
            return self.turn_right(abs(angle_diff * 180 / 3.14159))
```

### Step 3: Integration with Navigation and Perception

For a complete autonomous system, we need to integrate with navigation and perception systems:

```python
class IntegratedPerceptionSystem:
    def __init__(self):
        self.object_detector = None  # Initialize object detection model
        self.location_mapper = None  # Initialize location mapping
        self.obstacle_detector = None  # Initialize obstacle detection

    def update_environment_context(self) -> Dict[str, Any]:
        """Update environment information for the cognitive planner"""
        environment_info = {
            'detected_objects': self.detect_objects(),
            'obstacles': self.detect_obstacles(),
            'current_location': self.get_current_location(),
            'navigation_goals': self.get_navigation_goals()
        }

        return environment_info

    def detect_objects(self) -> list:
        """Detect objects in the environment"""
        # Implementation would use computer vision models
        # For simulation, return mock data
        return [
            {'type': 'cup', 'position': {'x': 1.5, 'y': 0.5}, 'distance': 1.6},
            {'type': 'chair', 'position': {'x': 2.0, 'y': -0.5}, 'distance': 2.1}
        ]

    def detect_obstacles(self) -> list:
        """Detect obstacles in the path"""
        # Implementation would use LIDAR or other sensors
        # For simulation, return mock data
        return [
            {'position': {'x': 0.8, 'y': 0.2}, 'size': 0.3},
            {'position': {'x': 1.2, 'y': -0.1}, 'size': 0.2}
        ]

    def get_current_location(self) -> Dict[str, float]:
        """Get current robot location"""
        # Implementation would use localization system
        # For simulation, return mock data
        return {'x': 0.0, 'y': 0.0, 'theta': 0.0}
```

## Complete System Integration

Now let's put it all together in the main system:

```python
def main(args=None):
    rclpy.init(args=args)

    # Create the orchestrator
    orchestrator = AutonomousHumanoidOrchestrator()

    # Create the perception system
    perception_system = IntegratedPerceptionSystem()

    # Update the cognitive planner with perception data
    def update_perception_data():
        """Periodically update perception data"""
        while rclpy.ok():
            env_info = perception_system.update_environment_context()
            orchestrator.cognitive_planner.update_environment(env_info)
            time.sleep(1.0)  # Update every second

    # Start perception update thread
    perception_thread = threading.Thread(target=update_perception_data, daemon=True)
    perception_thread.start()

    try:
        # Spin the orchestrator node
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Testing and Validation

### Unit Testing Components

Create tests for each component:

```python
import unittest
from unittest.mock import Mock, patch

class TestAutonomousHumanoid(unittest.TestCase):
    def setUp(self):
        self.planner = ContextAwarePlanner(api_key="test-key")

    def test_voice_processing(self):
        """Test voice processing with mock audio"""
        # Mock audio data processing
        mock_audio = "test audio data"
        result = self.planner.process_command_with_confidence(
            "test command", Mock()
        )

        self.assertIn('command', result)

    def test_action_generation(self):
        """Test action sequence generation"""
        command = "Go to the kitchen and bring me a cup"
        actions = self.planner.generate_action_sequence(command)

        self.assertIsInstance(actions, list)
        self.assertGreater(len(actions), 0)

    def test_contextual_planning(self):
        """Test planning with context"""
        command = "Go to the kitchen"
        context = {
            "robot_state": {"location": "living_room"},
            "environment": {"kitchen_location": {"x": 3.0, "y": 2.0}}
        }

        actions = self.planner.generate_contextual_action_sequence(command, context)

        self.assertIsInstance(actions, list)
        self.assertGreater(len(actions), 0)

if __name__ == '__main__':
    unittest.main()
```

### Integration Testing

Test the complete system with simulated commands:

```python
def test_complete_system():
    """Test the complete autonomous humanoid system"""

    # Simulate a complete command flow
    test_commands = [
        "Move forward 2 meters",
        "Turn left 90 degrees",
        "Go to the kitchen",
        "Bring me a cup"
    ]

    for command in test_commands:
        print(f"Testing command: {command}")

        # Test voice processing
        voice_result = voice_processor.process_command_with_confidence(
            command, voice_model
        )
        print(f"  Voice processing: {voice_result['text']}")

        # Test planning
        actions = cognitive_planner.generate_action_sequence(command)
        print(f"  Generated {len(actions)} actions")

        # Test execution (simulated)
        success = action_executor.execute_action_sequence(actions)
        print(f"  Execution success: {success}")
        print()

# Run integration test
test_complete_system()
```

## Error Handling and Recovery

Implement comprehensive error handling across the system:

```python
class ErrorHandlingSystem:
    def __init__(self):
        self.error_history = []
        self.recovery_strategies = {}

    def handle_voice_error(self, error: Exception, command: str):
        """Handle errors in voice processing"""
        self.log_error("voice_processing", error, command)

        # Recovery strategy
        return self.ask_for_repetition()

    def handle_planning_error(self, error: Exception, command: str):
        """Handle errors in cognitive planning"""
        self.log_error("cognitive_planning", error, command)

        # Recovery strategy
        return self.fallback_to_simple_plan(command)

    def handle_execution_error(self, error: Exception, action: Dict[str, Any]):
        """Handle errors in action execution"""
        self.log_error("action_execution", error, str(action))

        # Recovery strategy
        return self.attempt_action_recovery(action)

    def log_error(self, component: str, error: Exception, context: str):
        """Log error for analysis and debugging"""
        error_entry = {
            'timestamp': time.time(),
            'component': component,
            'error': str(error),
            'context': context
        }
        self.error_history.append(error_entry)

    def ask_for_repetition(self):
        """Ask user to repeat the command"""
        return {"action": "speak", "parameters": {"text": "Could you please repeat that?"}}

    def fallback_to_simple_plan(self, command: str):
        """Generate a simple plan when complex planning fails"""
        simple_actions = [
            {"action": "speak", "parameters": {"text": f"Processing: {command}"}},
            {"action": "speak", "parameters": {"text": "Command received, working on it"}}
        ]
        return simple_actions

    def attempt_action_recovery(self, action: Dict[str, Any]):
        """Attempt to recover from action execution failure"""
        recovery_actions = [
            {"action": "speak", "parameters": {"text": f"Retrying {action['action']}"}},
            action  # Retry the same action
        ]
        return recovery_actions
```

## Performance Optimization

Optimize the system for real-time operation:

```python
import asyncio
from concurrent.futures import ThreadPoolExecutor

class OptimizedAutonomousSystem:
    def __init__(self):
        # Use thread pool for I/O bound operations
        self.executor = ThreadPoolExecutor(max_workers=4)

        # Use asyncio for async operations
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        # Cache for frequently used plans
        self.plan_cache = {}

    async def process_command_async(self, command: str):
        """Process command asynchronously"""
        # Check cache first
        if command in self.plan_cache:
            return self.plan_cache[command]

        # Generate plan in thread pool
        plan = await self.loop.run_in_executor(
            self.executor,
            self.generate_plan_sync,
            command
        )

        # Cache the plan
        self.plan_cache[command] = plan

        return plan

    def generate_plan_sync(self, command: str):
        """Synchronous plan generation"""
        return self.cognitive_planner.generate_action_sequence(command)
```

## Practical Exercise

### Exercise 1: Complete System Integration

Implement the complete autonomous humanoid system:

1. Set up the three main components (voice processing, cognitive planning, action execution)
2. Create the orchestrator that connects all components
3. Implement the main loop that processes voice commands end-to-end
4. Add error handling and recovery mechanisms

### Exercise 2: Advanced Features

Extend the system with additional features:

1. Add multi-language support for voice commands
2. Implement learning capabilities to improve performance over time
3. Add gesture recognition as an alternative input method
4. Create a simple GUI for monitoring system status

## Troubleshooting Common Issues

### Voice Recognition Issues

- **Problem**: Poor voice recognition accuracy
- **Solution**: Check audio input quality, reduce background noise, use better Whisper model

### Planning Failures

- **Problem**: LLM generates incorrect action sequences
- **Solution**: Provide better context, use more specific prompts, implement validation

### Execution Problems

- **Problem**: Actions don't execute as expected
- **Solution**: Verify ROS 2 connections, check robot state, add action confirmation

## Summary

In this capstone project, we've integrated all the concepts from Module 4 to create a complete autonomous humanoid system. We've learned how to:

- Combine voice processing, cognitive planning, and action execution into a cohesive system
- Implement proper error handling and recovery mechanisms
- Create a robust orchestrator that manages the entire workflow
- Test and validate the integrated system
- Optimize performance for real-time operation

This project demonstrates the power of integrating vision, language, and action to create truly autonomous robotic systems. The skills learned in this module form the foundation for developing advanced AI-robotic systems that can understand and respond to human commands in natural ways.