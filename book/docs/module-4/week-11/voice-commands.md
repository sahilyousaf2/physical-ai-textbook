---
sidebar_label: 'Week 11: Voice Commands with OpenAI Whisper'
---

# Week 11: Voice Commands and Speech Processing with OpenAI Whisper

## Learning Objectives

By the end of this week, you will be able to:
- Integrate OpenAI Whisper for real-time speech recognition in humanoid robots
- Process and interpret voice commands for robotic control
- Implement voice activity detection and noise filtering
- Create context-aware speech processing pipelines
- Optimize speech recognition for noisy robotic environments
- Design multimodal interfaces combining voice and other inputs

## Overview

Voice interaction represents a crucial component of human-robot interaction, enabling natural communication between humans and humanoid robots. This week focuses on implementing OpenAI Whisper for robust speech recognition, specifically optimized for robotic applications where background noise, movement, and environmental factors can impact speech quality.

## OpenAI Whisper Fundamentals

### Whisper Architecture
- **Encoder**: Processes audio input using a Transformer architecture
- **Decoder**: Generates text output with language modeling capabilities
- **Multilingual Support**: Handles multiple languages in a single model
- **Robustness**: Designed to handle various audio conditions and accents
- **Efficiency**: Optimized for real-time processing on appropriate hardware

### Whisper Models for Robotics
- **tiny**: Fastest, suitable for real-time applications (75MB)
- **base**: Good balance of speed and accuracy (145MB)
- **small**: Better accuracy, moderate speed (485MB)
- **medium**: High accuracy, slower processing (1.5GB)
- **large**: Highest accuracy, best for complex tasks (3.0GB)

## Whisper Installation and Setup

### Basic Installation
```bash
# Install Whisper
pip install openai-whisper

# Install additional dependencies for audio processing
pip install torch torchaudio
pip install pyaudio sounddevice

# For faster processing with GPU
pip install openai-whisper[cuda]  # For CUDA support
```

### Docker-based Setup for Robotics
```dockerfile
FROM nvidia/cuda:11.8-devel-ubuntu20.04

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    ffmpeg \
    portaudio19-dev \
    python3-dev \
    git

# Install Python dependencies
RUN pip3 install torch torchaudio --index-url https://download.pytorch.org/whl/cu118
RUN pip3 install openai-whisper
RUN pip3 install pyaudio sounddevice

# Copy robot-specific code
COPY . /app
WORKDIR /app

CMD ["python3", "voice_robot.py"]
```

## Whisper Integration with ROS 2

### Voice Recognition Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
from geometry_msgs.msg import Twist
import whisper
import torch
import numpy as np
import pyaudio
import wave
import threading
import queue
import time

class WhisperVoiceNode(Node):
    def __init__(self):
        super().__init__('whisper_voice_node')

        # Publishers
        self.command_pub = self.create_publisher(String, '/voice_command', 10)
        self.robot_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parameters
        self.declare_parameter('model_size', 'base')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_size', 1024)
        self.declare_parameter('language', 'en')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')

        self.model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_size = self.get_parameter('chunk_size').value
        self.language = self.get_parameter('language').value
        self.device = self.get_parameter('device').value

        # Initialize Whisper model
        self.get_logger().info(f'Loading Whisper model: {self.model_size}')
        self.model = whisper.load_model(self.model_size, device=self.device)

        # Audio processing
        self.audio_queue = queue.Queue()
        self.is_listening = False
        self.audio_thread = None

        # Voice activity detection
        self.energy_threshold = 0.01
        self.silence_duration = 1.0  # seconds of silence to stop recording
        self.recording = False
        self.audio_buffer = []

        # Initialize audio stream
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            stream_callback=self.audio_callback
        )

        # Timer for processing audio
        self.process_timer = self.create_timer(0.1, self.process_audio)

        self.get_logger().info('Whisper Voice Node initialized')

    def audio_callback(self, in_data, frame_count, time_info, status):
        """Callback for audio input"""
        audio_data = np.frombuffer(in_data, dtype=np.float32)
        self.audio_queue.put(audio_data)
        return (in_data, pyaudio.paContinue)

    def process_audio(self):
        """Process audio data from queue"""
        audio_chunks = []

        # Collect audio data from queue
        while not self.audio_queue.empty():
            try:
                chunk = self.audio_queue.get_nowait()
                audio_chunks.append(chunk)
            except queue.Empty:
                break

        if audio_chunks:
            # Concatenate all audio chunks
            full_audio = np.concatenate(audio_chunks)

            # Perform voice activity detection
            if self.is_speech_detected(full_audio):
                if not self.recording:
                    self.start_recording()

                self.audio_buffer.extend(full_audio)
            else:
                if self.recording:
                    # Check if enough silence to stop recording
                    if len(self.audio_buffer) > self.sample_rate * 0.5:  # At least 0.5s of audio
                        self.stop_recording()
                    else:
                        # Not enough audio, continue buffering
                        self.audio_buffer.extend(full_audio)

    def is_speech_detected(self, audio_data):
        """Detect if speech is present in audio data"""
        # Calculate energy of audio signal
        energy = np.mean(np.abs(audio_data) ** 2)
        return energy > self.energy_threshold

    def start_recording(self):
        """Start recording audio for speech recognition"""
        self.recording = True
        self.audio_buffer = []
        self.get_logger().info('Started recording audio')

    def stop_recording(self):
        """Stop recording and process speech"""
        if len(self.audio_buffer) == 0:
            return

        # Convert to numpy array
        audio_array = np.array(self.audio_buffer, dtype=np.float32)

        # Process with Whisper
        threading.Thread(target=self.transcribe_audio, args=(audio_array,)).start()

        # Reset recording state
        self.recording = False
        self.audio_buffer = []

    def transcribe_audio(self, audio_array):
        """Transcribe audio using Whisper model"""
        try:
            # Convert audio to the format expected by Whisper
            audio_tensor = torch.from_numpy(audio_array).to(self.device)

            # Transcribe the audio
            result = self.model.transcribe(
                audio_tensor,
                language=self.language,
                fp16=(self.device == 'cuda')
            )

            transcription = result['text'].strip()

            if transcription:
                self.get_logger().info(f'Recognized: {transcription}')

                # Publish the recognized text
                cmd_msg = String()
                cmd_msg.data = transcription
                self.command_pub.publish(cmd_msg)

                # Process the command
                self.process_voice_command(transcription)

        except Exception as e:
            self.get_logger().error(f'Error in transcription: {e}')

    def process_voice_command(self, command_text):
        """Process the recognized voice command"""
        # Convert to lowercase for easier processing
        cmd_lower = command_text.lower()

        # Define command mappings
        command_mappings = {
            'move forward': self.move_forward,
            'go forward': self.move_forward,
            'move backward': self.move_backward,
            'go backward': self.move_backward,
            'turn left': self.turn_left,
            'turn right': self.turn_right,
            'stop': self.stop_robot,
            'halt': self.stop_robot,
            'dance': self.perform_dance,
            'hello': self.greet_user,
            'hi': self.greet_user,
        }

        # Find matching command
        for cmd_phrase, cmd_func in command_mappings.items():
            if cmd_phrase in cmd_lower:
                self.get_logger().info(f'Executing command: {cmd_phrase}')
                cmd_func()
                return

        # If no specific command found, publish as general command
        self.get_logger().info(f'Unknown command: {command_text}')

    def move_forward(self):
        """Move robot forward"""
        cmd = Twist()
        cmd.linear.x = 0.2  # m/s
        cmd.angular.z = 0.0
        self.robot_cmd_pub.publish(cmd)

    def move_backward(self):
        """Move robot backward"""
        cmd = Twist()
        cmd.linear.x = -0.2  # m/s
        cmd.angular.z = 0.0
        self.robot_cmd_pub.publish(cmd)

    def turn_left(self):
        """Turn robot left"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.3  # rad/s
        self.robot_cmd_pub.publish(cmd)

    def turn_right(self):
        """Turn robot right"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = -0.3  # rad/s
        self.robot_cmd_pub.publish(cmd)

    def stop_robot(self):
        """Stop robot movement"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.robot_cmd_pub.publish(cmd)

    def perform_dance(self):
        """Perform a simple dance routine"""
        self.get_logger().info('Performing dance routine...')
        # Implementation would involve complex movement patterns

    def greet_user(self):
        """Greet the user"""
        self.get_logger().info('Hello! How can I help you?')

    def destroy_node(self):
        """Clean up resources"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.audio:
            self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WhisperVoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Advanced Voice Processing for Robotics

### Noise Reduction and Audio Enhancement
```python
import numpy as np
from scipy import signal
import webrtcvad
import collections

class AudioProcessor:
    def __init__(self, sample_rate=16000):
        self.sample_rate = sample_rate
        self.frame_duration = 30  # ms
        self.frame_size = int(sample_rate * (self.frame_duration / 1000.0))

        # Initialize WebRTC VAD for voice activity detection
        self.vad = webrtcvad.Vad(2)  # Aggressiveness mode 2

        # Audio enhancement parameters
        self.noise_threshold = 0.001
        self.speech_threshold = 0.01

        # Ring buffer for audio frames
        self.frame_buffer = collections.deque(maxlen=30)  # 30 frames = 900ms

        # Noise estimation
        self.noise_estimate = None
        self.noise_frames = 0
        self.max_noise_frames = 100

    def preprocess_audio(self, audio_data):
        """Preprocess audio for better speech recognition"""
        # Apply pre-emphasis filter
        pre_emphasis = 0.97
        audio_data = np.append(audio_data[0], audio_data[1:] - pre_emphasis * audio_data[:-1])

        # Normalize audio
        audio_data = audio_data / (np.max(np.abs(audio_data)) + 1e-8)

        return audio_data

    def estimate_noise(self, audio_frame):
        """Estimate noise characteristics from audio"""
        if self.noise_frames < self.max_noise_frames:
            frame_energy = np.mean(np.abs(audio_frame) ** 2)

            if frame_energy < self.noise_threshold:
                if self.noise_estimate is None:
                    self.noise_estimate = frame_energy
                else:
                    # Update noise estimate with exponential smoothing
                    self.noise_estimate = 0.9 * self.noise_estimate + 0.1 * frame_energy

                self.noise_frames += 1

    def spectral_subtraction(self, audio_frame):
        """Apply spectral subtraction for noise reduction"""
        if self.noise_estimate is None:
            return audio_frame

        # Apply FFT
        fft_data = np.fft.rfft(audio_frame)
        magnitude = np.abs(fft_data)
        phase = np.angle(fft_data)

        # Estimate noise spectrum
        noise_magnitude = np.sqrt(self.noise_estimate) * np.ones_like(magnitude)

        # Apply spectral subtraction
        enhanced_magnitude = np.maximum(magnitude - 0.5 * noise_magnitude, 0.3 * magnitude)

        # Reconstruct signal
        enhanced_fft = enhanced_magnitude * np.exp(1j * phase)
        enhanced_audio = np.fft.irfft(enhanced_fft, len(audio_frame))

        return enhanced_audio.real

    def voice_activity_detection(self, audio_data):
        """Detect voice activity using WebRTC VAD"""
        # Convert to 16-bit PCM
        audio_int16 = (audio_data * 32767).astype(np.int16)

        # Split into frames
        frames = self.frame_generator(audio_int16)
        vad_results = []

        for frame in frames:
            is_speech = self.vad.is_speech(frame.tobytes(), self.sample_rate)
            vad_results.append(is_speech)

        # Simple voting mechanism
        speech_ratio = sum(vad_results) / len(vad_results) if vad_results else 0
        return speech_ratio > 0.3  # At least 30% of frames contain speech

    def frame_generator(self, audio_data):
        """Generate audio frames for VAD processing"""
        n = 0
        while n + self.frame_size <= len(audio_data):
            yield audio_data[n:n + self.frame_size]
            n += self.frame_size

    def enhance_audio_for_robot(self, audio_data):
        """Enhance audio specifically for robotic environment"""
        # Preprocess audio
        processed_audio = self.preprocess_audio(audio_data)

        # Estimate noise characteristics
        self.estimate_noise(processed_audio)

        # Apply noise reduction
        enhanced_audio = self.spectral_subtraction(processed_audio)

        # Apply voice activity detection
        is_speech = self.voice_activity_detection(enhanced_audio)

        return enhanced_audio, is_speech
```

## Context-Aware Voice Processing

### Context Manager for Voice Commands
```python
import json
from datetime import datetime, timedelta

class VoiceContextManager:
    def __init__(self):
        self.context_stack = []
        self.command_history = []
        self.user_preferences = {}
        self.robot_state = {}
        self.active_listening_context = None

    def set_context(self, context_name, context_data):
        """Set a new context for voice processing"""
        context = {
            'name': context_name,
            'data': context_data,
            'timestamp': datetime.now()
        }
        self.context_stack.append(context)

    def get_current_context(self):
        """Get the current active context"""
        if self.context_stack:
            return self.context_stack[-1]
        return None

    def pop_context(self):
        """Remove the current context"""
        if self.context_stack:
            return self.context_stack.pop()
        return None

    def add_command_to_history(self, command, result):
        """Add command to history for context awareness"""
        command_entry = {
            'command': command,
            'result': result,
            'timestamp': datetime.now()
        }
        self.command_history.append(command_entry)

        # Keep only recent history
        if len(self.command_history) > 50:
            self.command_history = self.command_history[-50:]

    def get_recent_commands(self, minutes=5):
        """Get commands from recent time period"""
        cutoff_time = datetime.now() - timedelta(minutes=minutes)
        recent_commands = [
            cmd for cmd in self.command_history
            if cmd['timestamp'] > cutoff_time
        ]
        return recent_commands

    def interpret_command_with_context(self, raw_command):
        """Interpret command considering current context"""
        current_context = self.get_current_context()
        recent_commands = self.get_recent_commands()

        # Apply context-specific interpretations
        if current_context:
            if current_context['name'] == 'navigation':
                # In navigation context, interpret relative commands
                if 'left' in raw_command.lower() or 'right' in raw_command.lower():
                    return self.interpret_navigation_command(raw_command)
            elif current_context['name'] == 'object_interaction':
                # In object interaction context, focus on manipulation commands
                return self.interpret_interaction_command(raw_command)

        # Default interpretation
        return self.interpret_general_command(raw_command)

    def interpret_navigation_command(self, command):
        """Interpret navigation-specific commands"""
        # Add navigation-specific context
        nav_interpretation = {
            'command': command,
            'context': 'navigation',
            'navigation_command': True
        }
        return nav_interpretation

    def interpret_interaction_command(self, command):
        """Interpret object interaction commands"""
        # Add interaction-specific context
        interaction_interpretation = {
            'command': command,
            'context': 'object_interaction',
            'interaction_command': True
        }
        return interaction_interpretation

    def interpret_general_command(self, command):
        """Interpret general commands"""
        general_interpretation = {
            'command': command,
            'context': 'general',
            'navigation_command': False,
            'interaction_command': False
        }
        return general_interpretation
```

## Voice Command Grammar and Intent Recognition

### Intent Recognition System
```python
import re
from typing import Dict, List, Tuple

class VoiceIntentRecognizer:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'navigation': [
                (r'move\s+(forward|backward|ahead)', 'move_direction'),
                (r'go\s+(forward|backward|ahead)', 'move_direction'),
                (r'walk\s+(forward|backward)', 'move_direction'),
                (r'turn\s+(left|right)', 'turn_direction'),
                (r'rotate\s+(left|right)', 'turn_direction'),
                (r'stop', 'stop'),
                (r'halt', 'stop'),
                (r'come\s+here', 'come_here'),
                (r'follow\s+me', 'follow'),
                (r'go\s+to\s+(?P<location>\w+)', 'go_to_location'),
            ],
            'manipulation': [
                (r'pick\s+up', 'pick_up'),
                (r'grab', 'pick_up'),
                (r'lift', 'pick_up'),
                (r'put\s+down', 'put_down'),
                (r'release', 'release'),
                (r'open', 'open_gripper'),
                (r'close', 'close_gripper'),
            ],
            'social': [
                (r'hello|hi|hey', 'greet'),
                (r'goodbye|bye', 'farewell'),
                (r'thank you|thanks', 'thank'),
                (r'please', 'please'),
                (r'sorry', 'apologize'),
            ],
            'information': [
                (r'what.*time', 'tell_time'),
                (r'what.*date', 'tell_date'),
                (r'how.*weather', 'weather_query'),
                (r'what.*your.*name', 'ask_name'),
                (r'what.*your.*purpose', 'ask_purpose'),
            ]
        }

        # Location keywords
        self.locations = {
            'kitchen', 'bedroom', 'living room', 'office', 'bathroom', 'hallway',
            'dining room', 'garden', 'garage', 'entrance', 'exit'
        }

    def recognize_intent(self, command: str) -> Tuple[str, str, Dict]:
        """Recognize intent from voice command"""
        command_lower = command.lower().strip()

        for intent_category, patterns in self.command_patterns.items():
            for pattern, intent_name in patterns:
                match = re.search(pattern, command_lower, re.IGNORECASE)
                if match:
                    # Extract parameters
                    params = match.groupdict()

                    # Handle location-specific commands
                    if 'location' in params:
                        location = params['location']
                        if location in self.locations:
                            params['location_type'] = 'known_location'
                        else:
                            params['location_type'] = 'unknown_location'

                    return intent_category, intent_name, params

        # If no pattern matches, return general intent
        return 'general', 'unknown', {'raw_command': command}

    def extract_entities(self, command: str) -> Dict:
        """Extract named entities from command"""
        entities = {
            'objects': [],
            'locations': [],
            'people': [],
            'actions': []
        }

        # Simple keyword-based entity extraction
        command_lower = command.lower()

        # Extract locations
        for location in self.locations:
            if location in command_lower:
                entities['locations'].append(location)

        # Extract action verbs
        action_patterns = [
            'move', 'go', 'turn', 'walk', 'run', 'stop', 'pick', 'grab', 'lift',
            'put', 'release', 'open', 'close', 'follow', 'come', 'get', 'take'
        ]

        for action in action_patterns:
            if action in command_lower:
                entities['actions'].append(action)

        return entities

    def classify_command(self, command: str) -> Dict:
        """Classify command with detailed information"""
        intent_category, intent_name, params = self.recognize_intent(command)
        entities = self.extract_entities(command)

        classification = {
            'command': command,
            'intent_category': intent_category,
            'intent_name': intent_name,
            'parameters': params,
            'entities': entities,
            'confidence': 0.9 if intent_name != 'unknown' else 0.1
        }

        return classification
```

## Practical Exercise: Voice-Enabled Humanoid Robot

### Step 1: Set up Whisper-based voice recognition
1. Install Whisper and dependencies
2. Configure audio input for your robot platform
3. Test basic speech recognition capabilities
4. Optimize for your specific hardware setup

### Step 2: Implement noise reduction
1. Add audio preprocessing pipeline
2. Implement voice activity detection
3. Add noise cancellation for robotic environments
4. Test in various acoustic conditions

### Step 3: Create command interpretation system
1. Implement intent recognition
2. Add context awareness
3. Create command-to-action mapping
4. Test with various voice commands

## Hands-On Project: Complete Voice Command System

Develop a complete voice command system that includes:
1. Whisper-based speech recognition
2. Audio preprocessing and noise reduction
3. Intent recognition and command interpretation
4. Context-aware command processing
5. Integration with robot control systems

## Assignment

1. Install and configure OpenAI Whisper for your robot platform
2. Implement a complete voice recognition pipeline
3. Add noise reduction and audio enhancement
4. Create an intent recognition system for robot commands
5. Test the system with various voice commands in different environments
6. Document the accuracy rates and optimization techniques used

## Resources

- [OpenAI Whisper Documentation](https://github.com/openai/whisper)
- [Speech Recognition in Robotics](https://www.cs.cmu.edu/~./rcm/papers/AAAI-08/icra08-speech.pdf)
- [Audio Processing for Robotics](https://ieeexplore.ieee.org/document/8202228)
- [Voice User Interfaces](https://dl.acm.org/doi/10.1145/3411764.3445712)

## Next Week Preview

Next week, we'll explore Large Language Model (LLM) integration for cognitive planning in humanoid robots. You'll learn to connect your voice recognition system with LLMs to enable sophisticated reasoning, planning, and decision-making capabilities that allow robots to understand complex, multi-step commands and execute appropriate action sequences.