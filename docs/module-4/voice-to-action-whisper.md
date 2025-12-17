# Voice-to-Action with OpenAI Whisper

This chapter covers how to implement voice processing systems using OpenAI Whisper to convert natural language commands into actionable tasks for humanoid robots.

## Overview

Voice interaction is a natural and intuitive way for humans to communicate with robots. In this chapter, we'll explore how to use OpenAI Whisper to process voice commands and convert them into actionable tasks that a humanoid robot can execute.

## Learning Objectives

By the end of this chapter, you will be able to:

- Set up and configure OpenAI Whisper for voice processing
- Process voice commands and convert them to text
- Extract actionable elements from natural language commands
- Integrate voice processing with robot control systems

## Introduction to OpenAI Whisper

OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) system trained on a large dataset of diverse audio. It can recognize speech in multiple languages and is particularly effective for converting voice commands into text that can be processed by robotic systems.

### Key Features of Whisper for Robotics

- **Multilingual Support**: Understands and processes commands in multiple languages
- **Robustness**: Works well in various acoustic environments
- **Accuracy**: High transcription accuracy for clear commands
- **Real-time Processing**: Can process audio streams in near real-time

## Setting Up Whisper for Robot Voice Commands

### Installation and Dependencies

To use OpenAI Whisper in your robotic system, you'll need to install the required dependencies:

```bash
pip install openai-whisper
```

For more efficient processing, you may also want to install additional dependencies:

```bash
pip install torch torchaudio
```

### Basic Whisper Implementation

Here's a basic implementation of Whisper for processing voice commands:

```python
import whisper
import torch
import pyaudio
import wave
import io
import numpy as np

class VoiceCommandProcessor:
    def __init__(self, model_size="base"):
        # Load the Whisper model
        self.model = whisper.load_model(model_size)

    def record_audio(self, duration=5, rate=16000):
        """Record audio from microphone for specified duration"""
        chunk = 1024
        format = pyaudio.paInt16
        channels = 1

        p = pyaudio.PyAudio()

        stream = p.open(format=format,
                        channels=channels,
                        rate=rate,
                        input=True,
                        frames_per_buffer=chunk)

        print("Recording...")
        frames = []

        for i in range(0, int(rate / chunk * duration)):
            data = stream.read(chunk)
            frames.append(data)

        print("Finished recording.")

        stream.stop_stream()
        stream.close()
        p.terminate()

        # Save to BytesIO for Whisper processing
        audio_data = b''.join(frames)
        return self.bytes_to_wav(audio_data, rate, channels)

    def bytes_to_wav(self, audio_bytes, rate, channels):
        """Convert raw audio bytes to WAV format for Whisper"""
        # Create a BytesIO buffer
        buffer = io.BytesIO()

        # Write WAV data to buffer
        with wave.open(buffer, 'wb') as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(2)  # 16-bit
            wav_file.setframerate(rate)
            wav_file.writeframes(audio_bytes)

        # Get WAV data as bytes and convert to numpy array
        wav_data = np.frombuffer(buffer.getvalue()[44:], dtype=np.int16).astype(np.float32) / 32768.0
        return wav_data

    def process_voice_command(self, audio_data):
        """Process audio with Whisper and return transcribed text"""
        result = self.model.transcribe(audio_data)
        return result["text"].strip()

# Example usage
processor = VoiceCommandProcessor()
audio_data = processor.record_audio(duration=5)
command_text = processor.process_voice_command(audio_data)
print(f"Recognized command: {command_text}")
```

## Processing Natural Language Commands

Once we have the transcribed text from Whisper, we need to extract actionable elements. This involves natural language understanding to identify what the user wants the robot to do.

### Command Parsing

Here's an example of how to parse common voice commands:

```python
import re

class CommandParser:
    def __init__(self):
        # Define patterns for common commands
        self.patterns = {
            'move_forward': r'go forward|move forward|step forward|walk forward',
            'move_backward': r'go backward|move backward|step backward|walk backward|back up',
            'turn_left': r'turn left|rotate left|pivot left',
            'turn_right': r'turn right|rotate right|pivot right',
            'raise_arm': r'raise your arm|lift your arm|raise right arm|lift right arm|raise left arm|lift left arm',
            'lower_arm': r'lower your arm|put your arm down|lower right arm|lower left arm',
            'stop': r'stop|halt|freeze|pause|wait',
            'wave': r'wave|wave hello|say hello|greet',
            'dance': r'dance|start dancing|move to music',
            'follow': r'follow me|come with me|follow',
            'sit': r'sit down|sit|take a seat',
            'stand': r'stand up|stand|get up'
        }

    def parse_command(self, text):
        """Parse the text and return the most likely command"""
        text_lower = text.lower()

        for command, pattern in self.patterns.items():
            if re.search(pattern, text_lower):
                return command

        return 'unknown'

    def extract_parameters(self, text):
        """Extract parameters from the command text"""
        params = {}

        # Extract direction if specified
        if 'left' in text.lower():
            params['direction'] = 'left'
        elif 'right' in text.lower():
            params['direction'] = 'right'

        # Extract duration if specified
        duration_match = re.search(r'for (\d+) seconds?', text.lower())
        if duration_match:
            params['duration'] = int(duration_match.group(1))

        # Extract distance if specified
        distance_match = re.search(r'(\d+(?:\.\d+)?) (meters?|steps?)', text.lower())
        if distance_match:
            params['distance'] = float(distance_match.group(1))
            params['unit'] = distance_match.group(2)

        return params

# Example usage
parser = CommandParser()
command = parser.parse_command("Please go forward for 3 meters")
params = parser.extract_parameters("Please go forward for 3 meters")
print(f"Command: {command}, Parameters: {params}")
```

## Integrating with ROS 2

Now let's integrate our voice processing system with ROS 2 to control a humanoid robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from builtin_interfaces.msg import Duration

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        # Create publisher for robot movement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create subscriber for voice commands
        self.voice_subscriber = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        # Initialize voice processor and parser
        self.voice_processor = VoiceCommandProcessor()
        self.command_parser = CommandParser()

        self.get_logger().info('Voice control node initialized')

    def voice_command_callback(self, msg):
        """Process incoming voice command"""
        command_text = msg.data
        self.get_logger().info(f'Received voice command: {command_text}')

        # Parse the command
        command = self.command_parser.parse_command(command_text)
        params = self.command_parser.extract_parameters(command_text)

        # Execute the appropriate action
        self.execute_command(command, params)

    def execute_command(self, command, params):
        """Execute the parsed command on the robot"""
        twist_msg = Twist()

        if command == 'move_forward':
            twist_msg.linear.x = 0.5  # Adjust speed as needed
            duration = params.get('duration', 1.0)
        elif command == 'move_backward':
            twist_msg.linear.x = -0.5
            duration = params.get('duration', 1.0)
        elif command == 'turn_left':
            twist_msg.angular.z = 0.5
            duration = params.get('duration', 1.0)
        elif command == 'turn_right':
            twist_msg.angular.z = -0.5
            duration = params.get('duration', 1.0)
        elif command == 'stop':
            # Stop the robot by sending zero velocities
            pass
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        # Publish the command
        self.cmd_vel_publisher.publish(twist_msg)

        # Stop after the specified duration if needed
        if command != 'stop':
            timer = self.create_timer(duration, lambda: self.stop_robot())

    def stop_robot(self):
        """Stop the robot by sending zero velocities"""
        twist_msg = Twist()
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()

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

## Advanced Voice Processing Techniques

### Handling Ambiguity

Voice commands can sometimes be ambiguous or unclear. Here's how to handle such situations:

```python
class AdvancedCommandProcessor:
    def __init__(self):
        self.command_parser = CommandParser()
        self.confidence_threshold = 0.7

    def process_command_with_confidence(self, audio_data, whisper_model):
        """Process command with confidence scoring"""
        result = whisper_model.transcribe(audio_data, return_segments=True)

        # Get the main transcription
        text = result["text"].strip()

        # Calculate confidence based on various factors
        confidence = self.calculate_confidence(text, result)

        if confidence < self.confidence_threshold:
            return {
                'command': 'unclear',
                'text': text,
                'confidence': confidence,
                'alternatives': self.get_alternative_interpretations(text)
            }

        command = self.command_parser.parse_command(text)
        params = self.command_parser.extract_parameters(text)

        return {
            'command': command,
            'text': text,
            'confidence': confidence,
            'parameters': params
        }

    def calculate_confidence(self, text, result):
        """Calculate confidence score for the transcription"""
        # Simple confidence calculation based on text length and common words
        if len(text) < 3:
            return 0.1

        # Check for common filler words that might indicate unclear audio
        filler_words = ['um', 'uh', 'er', 'ah']
        filler_count = sum(1 for word in filler_words if word in text.lower())

        # Base confidence on text length and clarity
        base_confidence = min(len(text) / 50, 1.0)  # Max confidence for longer texts

        # Reduce confidence if many filler words are present
        confidence = base_confidence - (filler_count * 0.1)

        return max(0.1, confidence)  # Minimum confidence of 0.1

    def get_alternative_interpretations(self, text):
        """Provide alternative interpretations of unclear commands"""
        alternatives = []

        # Try to identify possible variations of the command
        if 'forward' in text.lower():
            alternatives.append(text.replace('forward', 'ahead'))
        if 'backward' in text.lower():
            alternatives.append(text.replace('backward', 'back'))

        return alternatives
```

## Practical Exercise

### Exercise 1: Basic Voice Command Recognition

Create a simple system that:
1. Records voice input from the microphone
2. Uses Whisper to transcribe the command
3. Parses the command to identify robot actions
4. Prints the recognized command to the console

### Exercise 2: Integration Challenge

Extend the basic system to:
1. Control a simulated humanoid robot in Gazebo
2. Handle at least 5 different voice commands
3. Provide feedback when commands are recognized
4. Include error handling for unclear commands

## Summary

In this chapter, we've explored how to implement voice processing systems using OpenAI Whisper. We've learned how to:

- Set up and configure Whisper for voice processing
- Process voice commands and convert them to text
- Extract actionable elements from natural language commands
- Integrate voice processing with ROS 2 robot control systems
- Handle ambiguous or unclear commands

The next chapter will build on these concepts by exploring how to use large language models for cognitive planning, translating high-level commands into detailed action sequences.