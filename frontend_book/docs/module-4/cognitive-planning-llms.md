# Cognitive Planning with LLMs

This chapter covers how to implement cognitive planning systems using Large Language Models (LLMs) to translate high-level commands into ROS 2 action sequences for humanoid robots.

## Overview

Cognitive planning is the process of taking high-level, abstract commands and breaking them down into specific, executable actions. In this chapter, we'll explore how to leverage the reasoning capabilities of Large Language Models to create intelligent planning systems for humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:

- Integrate LLMs with robotic systems for cognitive planning
- Translate high-level commands into detailed ROS 2 action sequences
- Handle complex and ambiguous commands with LLM-based reasoning
- Implement error handling and fallback strategies for planning failures

## Introduction to LLM-Based Cognitive Planning

Large Language Models have demonstrated remarkable capabilities in understanding natural language and generating coherent, contextually appropriate responses. In robotics, we can leverage these capabilities to create cognitive planning systems that interpret high-level commands and generate detailed action sequences.

### Key Benefits of LLM-Based Planning

- **Natural Language Understanding**: LLMs can interpret commands expressed in natural language
- **Reasoning Capabilities**: LLMs can reason about the steps needed to achieve goals
- **Context Awareness**: LLMs can consider context when planning actions
- **Flexibility**: LLMs can adapt to new types of commands without explicit programming

## Setting Up LLM Integration

### Installation and Dependencies

To use LLMs for cognitive planning, you'll need to install the required dependencies:

```bash
pip install openai
# or for other providers:
pip install langchain langchain-openai  # for OpenAI
pip install langchain-anthropic         # for Anthropic
pip install langchain-google            # for Google models
```

### Basic LLM Integration

Here's a basic implementation of LLM integration for cognitive planning:

```python
import openai
from typing import Dict, List, Any
import json

class CognitivePlanner:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        openai.api_key = api_key
        self.model = model

    def generate_action_sequence(self, command: str, robot_capabilities: List[str] = None) -> List[Dict[str, Any]]:
        """
        Generate a sequence of actions from a high-level command using an LLM
        """
        if robot_capabilities is None:
            robot_capabilities = [
                "move_forward", "move_backward", "turn_left", "turn_right",
                "raise_arm", "lower_arm", "wave", "grasp_object", "release_object",
                "navigate_to_location", "detect_object", "speak"
            ]

        prompt = f"""
        You are a cognitive planning system for a humanoid robot. Your task is to break down high-level commands into specific, executable actions.

        Robot capabilities: {', '.join(robot_capabilities)}

        Command: {command}

        Please provide a sequence of actions that the robot should perform to fulfill this command. Each action should be a dictionary with:
        - "action": the specific action to perform
        - "parameters": any required parameters for the action
        - "description": a brief description of why this action is needed

        Return the sequence as a JSON array of action dictionaries.
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1  # Low temperature for more consistent planning
            )

            # Extract the JSON response
            content = response.choices[0].message.content.strip()

            # Clean up the response to extract JSON
            if content.startswith("```json"):
                content = content[7:content.rfind("```")]
            elif content.startswith("```"):
                content = content[3:content.rfind("```")]

            action_sequence = json.loads(content)
            return action_sequence

        except Exception as e:
            print(f"Error generating action sequence: {e}")
            return self.get_default_fallback_actions(command)

    def get_default_fallback_actions(self, command: str) -> List[Dict[str, Any]]:
        """
        Fallback method when LLM fails to generate actions
        """
        return [
            {
                "action": "speak",
                "parameters": {"text": f"I'm not sure how to execute: {command}"},
                "description": "Inform user of inability to execute command"
            }
        ]
```

## Advanced Planning with Context

### Context-Aware Planning

For more sophisticated planning, we can provide context about the robot's environment and state:

```python
class ContextAwarePlanner(CognitivePlanner):
    def __init__(self, api_key: str, model: str = "gpt-4"):
        super().__init__(api_key, model)
        self.robot_state = {
            "location": "starting_position",
            "battery_level": 100,
            "arm_position": "neutral",
            "gripper_status": "open"
        }
        self.environment = {
            "objects": [],
            "obstacles": [],
            "navigation_goals": []
        }

    def update_robot_state(self, state_updates: Dict[str, Any]):
        """Update the robot's internal state"""
        self.robot_state.update(state_updates)

    def update_environment(self, env_updates: Dict[str, Any]):
        """Update the environment information"""
        self.environment.update(env_updates)

    def generate_contextual_action_sequence(self, command: str, additional_context: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Generate action sequence with context about robot state and environment
        """
        context = {
            "robot_state": self.robot_state,
            "environment": self.environment
        }

        if additional_context:
            context.update(additional_context)

        prompt = f"""
        You are a cognitive planning system for a humanoid robot. Your task is to break down high-level commands into specific, executable actions.

        Current robot state: {json.dumps(self.robot_state, indent=2)}
        Current environment: {json.dumps(self.environment, indent=2)}

        Additional context: {json.dumps(additional_context or {}, indent=2) if additional_context else 'None'}

        Command: {command}

        Please provide a sequence of actions that the robot should perform to fulfill this command, taking into account the current state and environment. Each action should be a dictionary with:
        - "action": the specific action to perform
        - "parameters": any required parameters for the action
        - "description": a brief description of why this action is needed
        - "expected_outcome": what should happen after this action

        Return the sequence as a JSON array of action dictionaries.
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content.strip()
            if content.startswith("```json"):
                content = content[7:content.rfind("```")]
            elif content.startswith("```"):
                content = content[3:content.rfind("```")]

            action_sequence = json.loads(content)
            return action_sequence

        except Exception as e:
            print(f"Error generating contextual action sequence: {e}")
            return self.get_default_fallback_actions(command)
```

## Integration with ROS 2 Action System

Let's integrate our cognitive planner with ROS 2's action system:

```python
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import threading
import time

from example_interfaces.action import ExecutePlan  # Custom action definition

class CognitivePlanningNode(Node):
    def __init__(self):
        super().__init__('cognitive_planning_node')

        # Initialize the cognitive planner
        self.planner = ContextAwarePlanner(api_key="your-api-key")

        # Create action server for executing plans
        self._action_server = ActionServer(
            self,
            ExecutePlan,
            'execute_plan',
            self.execute_plan_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # Create publisher for plan execution status
        self.status_publisher = self.create_publisher(String, 'plan_status', 10)

        # Create subscriber for environment updates
        self.env_subscriber = self.create_subscription(
            String,
            'environment_updates',
            self.environment_callback,
            10
        )

        self.get_logger().info('Cognitive planning node initialized')

    def goal_callback(self, goal_request):
        """Accept or reject goal request"""
        self.get_logger().info(f'Received plan execution request: {goal_request.command}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject cancel request"""
        self.get_logger().info('Received request to cancel plan execution')
        return CancelResponse.ACCEPT

    def environment_callback(self, msg):
        """Update environment information"""
        try:
            env_data = json.loads(msg.data)
            self.planner.update_environment(env_data)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid environment update message')

    def execute_plan_callback(self, goal_handle):
        """Execute the cognitive plan"""
        feedback_msg = ExecutePlan.Feedback()
        result = ExecutePlan.Result()

        command = goal_handle.request.command
        self.get_logger().info(f'Executing plan for command: {command}')

        # Generate action sequence using LLM
        action_sequence = self.planner.generate_contextual_action_sequence(command)

        if not action_sequence:
            result.success = False
            result.message = "Failed to generate action sequence"
            goal_handle.succeed()
            return result

        # Execute each action in the sequence
        for i, action in enumerate(action_sequence):
            if goal_handle.is_cancel_requested:
                result.success = False
                result.message = "Plan execution cancelled"
                goal_handle.canceled()
                return result

            # Publish feedback
            feedback_msg.current_action = f"Executing: {action['action']}"
            feedback_msg.progress = (i + 1) / len(action_sequence) * 100.0
            goal_handle.publish_feedback(feedback_msg)

            # Execute the action
            success = self.execute_single_action(action)

            if not success:
                result.success = False
                result.message = f"Action failed: {action['action']}"
                goal_handle.abort()
                return result

            time.sleep(0.1)  # Small delay between actions

        result.success = True
        result.message = "Plan executed successfully"
        goal_handle.succeed()
        return result

    def execute_single_action(self, action: Dict[str, Any]) -> bool:
        """Execute a single action on the robot"""
        action_type = action['action']
        parameters = action.get('parameters', {})

        try:
            if action_type == 'move_forward':
                return self.move_robot('forward', parameters.get('distance', 1.0))
            elif action_type == 'turn_left':
                return self.rotate_robot('left', parameters.get('angle', 90.0))
            elif action_type == 'navigate_to_location':
                return self.navigate_to_location(parameters.get('x', 0.0), parameters.get('y', 0.0))
            elif action_type == 'speak':
                return self.speak(parameters.get('text', ''))
            elif action_type == 'detect_object':
                return self.detect_object(parameters.get('object_type', 'any'))
            else:
                self.get_logger().warn(f'Unknown action type: {action_type}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error executing action {action_type}: {e}')
            return False

    def move_robot(self, direction: str, distance: float) -> bool:
        """Move the robot in the specified direction"""
        # Implementation for moving the robot
        self.get_logger().info(f'Moving {direction} for {distance} meters')
        return True

    def rotate_robot(self, direction: str, angle: float) -> bool:
        """Rotate the robot"""
        # Implementation for rotating the robot
        self.get_logger().info(f'Rotating {direction} by {angle} degrees')
        return True

    def navigate_to_location(self, x: float, y: float) -> bool:
        """Navigate to a specific location"""
        # Implementation for navigation
        self.get_logger().info(f'Navigating to location ({x}, {y})')
        return True

    def speak(self, text: str) -> bool:
        """Make the robot speak"""
        # Implementation for text-to-speech
        self.get_logger().info(f'Robot says: {text}')
        return True

    def detect_object(self, object_type: str) -> bool:
        """Detect objects of a specific type"""
        # Implementation for object detection
        self.get_logger().info(f'Detecting {object_type} objects')
        return True

def main(args=None):
    rclpy.init(args=args)
    node = CognitivePlanningNode()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Handling Complex and Ambiguous Commands

### Command Clarification System

Sometimes LLMs may need to ask for clarification when commands are ambiguous:

```python
class ClarificationPlanner(CognitivePlanner):
    def __init__(self, api_key: str, model: str = "gpt-4"):
        super().__init__(api_key, model)
        self.pending_clarifications = {}

    def generate_action_sequence_with_clarification(self, command: str, robot_capabilities: List[str] = None) -> Dict[str, Any]:
        """
        Generate action sequence, with clarification if needed
        """
        prompt = f"""
        You are a cognitive planning system for a humanoid robot. Analyze this command to determine if clarification is needed.

        Command: {command}

        Robot capabilities: {', '.join(robot_capabilities or [])}

        If the command is clear and actionable, return the action sequence as a JSON array.
        If clarification is needed, return a JSON object with:
        - "needs_clarification": true
        - "clarification_question": a specific question to ask the user
        - "possible_interpretations": a list of possible interpretations

        Be conservative - if there's any ambiguity that could lead to incorrect actions, ask for clarification.
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content.strip()
            if content.startswith("```json"):
                content = content[7:content.rfind("```")]
            elif content.startswith("```"):
                content = content[3:content.rfind("```")]

            result = json.loads(content)

            # Check if clarification is needed
            if isinstance(result, dict) and result.get("needs_clarification"):
                # Store the clarification request for later
                request_id = f"clarify_{int(time.time())}"
                self.pending_clarifications[request_id] = {
                    "command": command,
                    "question": result["clarification_question"],
                    "interpretations": result["possible_interpretations"]
                }
                return {
                    "needs_clarification": True,
                    "request_id": request_id,
                    "question": result["clarification_question"],
                    "interpretations": result["possible_interpretations"]
                }
            else:
                # Return the action sequence
                return {
                    "needs_clarification": False,
                    "action_sequence": result
                }

        except Exception as e:
            print(f"Error in clarification planning: {e}")
            return {
                "needs_clarification": False,
                "action_sequence": self.get_default_fallback_actions(command)
            }

    def process_clarification_response(self, request_id: str, user_response: str):
        """
        Process user's response to a clarification question
        """
        if request_id not in self.pending_clarifications:
            return {"error": "Invalid clarification request ID"}

        original_command = self.pending_clarifications[request_id]["command"]

        # Combine original command with clarification
        clarified_command = f"{original_command} [Clarification: {user_response}]"

        # Generate action sequence for the clarified command
        action_sequence = self.generate_action_sequence(clarified_command)

        # Remove the pending clarification
        del self.pending_clarifications[request_id]

        return {
            "action_sequence": action_sequence,
            "clarified_command": clarified_command
        }
```

## Error Handling and Fallback Strategies

### Robust Planning with Error Recovery

Implementing error handling is crucial for reliable robot operation:

```python
class RobustPlanner(CognitivePlanner):
    def __init__(self, api_key: str, model: str = "gpt-4"):
        super().__init__(api_key, model)
        self.action_history = []

    def execute_plan_with_error_recovery(self, command: str, max_retries: int = 3):
        """
        Execute a plan with error recovery capabilities
        """
        # Generate initial plan
        action_sequence = self.generate_action_sequence(command)

        for attempt in range(max_retries):
            success, failed_action_idx = self.execute_action_sequence(action_sequence)

            if success:
                return {"success": True, "attempts": attempt + 1}

            # If failed, generate a recovery plan
            if failed_action_idx < len(action_sequence):
                failed_action = action_sequence[failed_action_idx]
                recovery_plan = self.generate_recovery_plan(command, failed_action, attempt)

                if recovery_plan:
                    # Insert recovery actions before the failed action
                    action_sequence = recovery_plan + action_sequence[failed_action_idx:]

        return {"success": False, "attempts": max_retries}

    def generate_recovery_plan(self, original_command: str, failed_action: Dict[str, Any], attempt: int):
        """
        Generate a recovery plan when an action fails
        """
        prompt = f"""
        You are a recovery planning system for a humanoid robot. An action failed during plan execution.

        Original command: {original_command}
        Failed action: {json.dumps(failed_action)}
        Attempt number: {attempt + 1}

        Generate a recovery plan to address the failure and continue with the original goal.
        Return the recovery plan as a JSON array of actions to execute before continuing with the original plan.
        """

        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.1
            )

            content = response.choices[0].message.content.strip()
            if content.startswith("```json"):
                content = content[7:content.rfind("```")]
            elif content.startswith("```"):
                content = content[3:content.rfind("```")]

            return json.loads(content)

        except Exception as e:
            print(f"Error generating recovery plan: {e}")
            return []

    def execute_action_sequence(self, action_sequence: List[Dict[str, Any]]):
        """
        Execute a sequence of actions, returning success status and failed action index
        """
        for i, action in enumerate(action_sequence):
            success = self.execute_single_action_with_monitoring(action)

            if not success:
                return False, i  # Return False and the index of the failed action

        return True, -1  # Return True and -1 if all actions succeeded
```

## Practical Exercise

### Exercise 1: Basic Cognitive Planning

Create a simple cognitive planning system that:
1. Takes a high-level command (e.g., "Go to the kitchen and bring me a cup")
2. Uses an LLM to break it down into specific actions
3. Prints the action sequence to the console

### Exercise 2: Context-Aware Planning

Extend the basic system to:
1. Maintain robot state information (location, battery, etc.)
2. Incorporate environmental context
3. Generate more appropriate action sequences based on context
4. Handle ambiguous commands by asking for clarification

## Summary

In this chapter, we've explored how to implement cognitive planning systems using Large Language Models. We've learned how to:

- Integrate LLMs with robotic systems for cognitive planning
- Translate high-level commands into detailed ROS 2 action sequences
- Handle complex and ambiguous commands with LLM-based reasoning
- Implement error handling and fallback strategies for planning failures
- Create context-aware planning systems that consider robot state and environment

The next chapter will bring together all the concepts from this module in a comprehensive capstone project: creating an autonomous humanoid system that combines voice processing, cognitive planning, and robot execution.