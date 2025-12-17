# Quickstart: Module 4 - Vision-Language-Action

## Overview
This quickstart guide provides the essential steps to implement and understand Module 4: Vision-Language-Action in the Docusaurus-based educational book on AI and robotics.

## Prerequisites

### Software Requirements
- Python 3.8 or higher
- ROS 2 (recommended: Humble Hawksbill or later)
- Node.js and npm for Docusaurus
- OpenAI API key (for LLM examples)

### Educational Prerequisites
- Completion of Module 1: ROS 2 Fundamentals
- Basic understanding of Python programming
- Familiarity with Docusaurus documentation framework

## Setup Instructions

### 1. Environment Setup
```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Install Python dependencies
pip install openai-whisper
pip install openai
pip install torch torchaudio  # For Whisper processing
pip install rclpy  # For ROS 2 integration

# Install audio processing dependencies
pip install pyaudio
pip install numpy
```

### 2. Docusaurus Setup
```bash
# Navigate to frontend book directory
cd frontend_book

# Install dependencies
npm install

# Start development server
npm start
```

### 3. Configuration
Create a `.env` file in the project root with:
```env
OPENAI_API_KEY=your_openai_api_key_here
```

## Key Components

### 1. Voice Processing Pipeline
The voice processing component converts spoken commands into actionable text:

```python
import whisper
import pyaudio

class VoiceCommandProcessor:
    def __init__(self, model_size="base"):
        self.model = whisper.load_model(model_size)

    def process_audio(self, audio_data):
        result = self.model.transcribe(audio_data)
        return result["text"].strip()
```

### 2. Cognitive Planning Engine
The cognitive planning component translates high-level commands into action sequences:

```python
import openai
import json

class CognitivePlanner:
    def __init__(self, api_key):
        openai.api_key = api_key

    def generate_action_sequence(self, command):
        prompt = f"""
        You are a cognitive planning system for a humanoid robot.
        Command: {command}

        Please provide a sequence of actions as a JSON array.
        Each action should have: "action", "parameters", "description"
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1
        )

        content = response.choices[0].message.content.strip()
        if content.startswith("```json"):
            content = content[7:content.rfind("```")]

        return json.loads(content)
```

### 3. Action Execution System
The action execution component carries out the planned actions on the robot:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ActionExecutor(Node):
    def __init__(self):
        super().__init__('action_executor')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def execute_action(self, action):
        if action['action'] == 'move_forward':
            self.move_forward(action['parameters'].get('distance', 1.0))

    def move_forward(self, distance):
        # Implementation for moving the robot
        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        self.cmd_vel_publisher.publish(twist_msg)
```

## Implementation Steps

### 1. Voice-to-Action Chapter
1. Implement the voice processing pipeline
2. Create the command parsing system
3. Integrate with ROS 2 for basic movement commands
4. Test with various voice commands

### 2. Cognitive Planning Chapter
1. Set up LLM integration for planning
2. Create context-aware planning system
3. Implement error handling and fallback strategies
4. Test with complex high-level commands

### 3. Capstone Project
1. Integrate all components into a unified system
2. Create the orchestrator that manages the workflow
3. Implement comprehensive error handling
4. Test end-to-end functionality

## Running Examples

### Basic Voice Command Processing
```bash
# Start the voice processing example
python examples/voice_processing.py
```

### Cognitive Planning Test
```bash
# Test the cognitive planning system
python examples/cognitive_planning.py "Go to the kitchen and bring me a cup"
```

### Full System Integration
```bash
# Run the complete autonomous humanoid system
python examples/autonomous_humanoid.py
```

## Common Issues and Solutions

### Audio Input Issues
- **Problem**: Poor voice recognition accuracy
- **Solution**: Check microphone permissions, reduce background noise, use better Whisper model

### LLM Integration Issues
- **Problem**: API rate limits or authentication failures
- **Solution**: Verify API key, implement retry logic, use appropriate rate limiting

### ROS 2 Connection Issues
- **Problem**: Robot control commands not executing
- **Solution**: Verify ROS 2 network setup, check topic names, ensure proper node initialization

## Next Steps

After completing this module, you should be able to:
1. Implement voice processing systems using OpenAI Whisper
2. Create cognitive planning systems with LLMs
3. Integrate voice, planning, and execution in a complete system
4. Extend the system with additional capabilities

Continue to the individual chapter documentation for detailed implementation guides.