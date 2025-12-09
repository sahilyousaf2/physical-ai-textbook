---
sidebar_label: 'Week 12: LLM Cognitive Planning and Decision Making'
---

# Week 12: LLM Cognitive Planning and Decision Making

## Learning Objectives

By the end of this week, you will be able to:
- Integrate Large Language Models (LLMs) with humanoid robot systems
- Implement cognitive planning using LLMs for complex task decomposition
- Design prompt engineering strategies for robotic applications
- Create decision-making frameworks that combine LLM reasoning with robot control
- Develop multimodal interfaces that combine vision, language, and action
- Evaluate and validate LLM-based planning systems for safety and reliability

## Overview

Large Language Models represent a breakthrough in artificial intelligence that enables humanoid robots to understand complex natural language commands, reason about their environment, and plan sophisticated action sequences. This week focuses on integrating LLMs with robotic systems to create cognitive agents capable of high-level reasoning, planning, and decision-making.

## LLM Fundamentals for Robotics

### Key LLM Architectures
- **Transformer Architecture**: Attention mechanisms for understanding context
- **Instruction Tuning**: Models fine-tuned for following instructions
- **Reinforcement Learning from Human Feedback (RLHF)**: Models aligned with human preferences
- **Multimodal Capabilities**: Models that process both text and images
- **Chain-of-Thought Reasoning**: Models that show their reasoning process

### LLM Types for Robotics
- **OpenAI GPT Models**: Proprietary models with strong reasoning capabilities
- **Anthropic Claude**: Models designed with safety and helpfulness in mind
- **Open Source Models**: Llama, Mistral, Gemma for on-premises deployment
- **Specialized Models**: Models fine-tuned for robotics applications

## LLM Integration Architecture

### Cognitive Planning Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge
import openai
import json
import time
from typing import Dict, List, Any
import numpy as np

class LLMCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')

        # Publishers
        self.action_plan_pub = self.create_publisher(String, '/action_plan', 10)
        self.navigation_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.robot_cmd_pub = self.create_publisher(String, '/robot_command', 10)

        # Subscribers
        self.voice_cmd_sub = self.create_subscription(
            String, '/voice_command', self.voice_command_callback, 10)
        self.vision_sub = self.create_subscription(
            Image, '/camera/image_raw', self.vision_callback, 10)

        # Parameters
        self.declare_parameter('llm_model', 'gpt-4-turbo')
        self.declare_parameter('api_key', '')
        self.declare_parameter('max_tokens', 1000)
        self.declare_parameter('temperature', 0.3)

        self.llm_model = self.get_parameter('llm_model').value
        self.api_key = self.get_parameter('api_key').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.temperature = self.get_parameter('temperature').value

        # Initialize OpenAI client
        if self.api_key:
            openai.api_key = self.api_key
            self.client = openai.OpenAI(api_key=self.api_key)
        else:
            self.get_logger().warn('No API key provided, using mock responses')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Robot state and context
        self.robot_pose = None
        self.environment_description = ""
        self.object_locations = {}
        self.action_history = []

        # Vision processing
        self.last_image = None
        self.image_processed = False

        self.get_logger().info('LLM Cognitive Planner initialized')

    def voice_command_callback(self, msg):
        """Process voice command through LLM cognitive planner"""
        command = msg.data
        self.get_logger().info(f'Received voice command: {command}')

        # Get robot state and environment context
        context = self.build_context(command)

        # Generate action plan using LLM
        action_plan = self.generate_action_plan(context)

        if action_plan:
            # Execute the action plan
            self.execute_action_plan(action_plan)

    def vision_callback(self, msg):
        """Process camera image for environment understanding"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_image = cv_image
            self.image_processed = True
        except Exception as e:
            self.get_logger().error(f'Error processing vision: {e}')

    def build_context(self, command):
        """Build context for LLM including robot state and environment"""
        context = {
            "robot_state": {
                "current_pose": self.robot_pose,
                "capabilities": [
                    "navigation",
                    "object manipulation",
                    "speech",
                    "vision"
                ],
                "current_location": "unknown" if not self.robot_pose else f"({self.robot_pose.position.x:.2f}, {self.robot_pose.position.y:.2f})"
            },
            "environment": {
                "description": self.environment_description,
                "objects": self.object_locations,
                "recent_observations": self.get_recent_observations()
            },
            "command": command,
            "action_history": self.action_history[-5:]  # Last 5 actions
        }

        return context

    def generate_action_plan(self, context):
        """Generate action plan using LLM"""
        # Create system message for the LLM
        system_message = """
        You are a cognitive planner for a humanoid robot. Your role is to:
        1. Understand the user's command in the context of the robot's current state
        2. Break down complex commands into simple, executable actions
        3. Consider the robot's capabilities and environment constraints
        4. Generate a sequence of actions that the robot can execute
        5. Ensure safety and feasibility of the plan

        Available actions:
        - NAVIGATE_TO(location): Move robot to specified location
        - PICK_UP(object): Pick up an object
        - PLACE(object, location): Place an object at a location
        - SPEAK(text): Make the robot speak
        - GREET: Make the robot greet
        - WAIT(duration): Wait for specified duration
        - SCAN_ENVIRONMENT: Use vision to observe surroundings
        - ASK_FOR_HELP: Request human assistance

        Response format: JSON with 'plan' as array of actions and 'reasoning' as explanation.
        """

        # Create user message with context
        user_message = f"""
        Robot Context:
        {json.dumps(context, indent=2)}

        Please generate an action plan to fulfill the user's command.
        """

        try:
            if hasattr(self, 'client'):
                response = self.client.chat.completions.create(
                    model=self.llm_model,
                    messages=[
                        {"role": "system", "content": system_message},
                        {"role": "user", "content": user_message}
                    ],
                    max_tokens=self.max_tokens,
                    temperature=self.temperature
                )

                response_text = response.choices[0].message.content
            else:
                # Mock response for testing
                response_text = """
                {
                    "reasoning": "User wants robot to bring a cup from kitchen to living room. Robot needs to navigate to kitchen, identify cup, pick it up, then navigate to living room and place it down.",
                    "plan": [
                        {"action": "NAVIGATE_TO", "arguments": {"location": "kitchen"}},
                        {"action": "SCAN_ENVIRONMENT", "arguments": {}},
                        {"action": "PICK_UP", "arguments": {"object": "cup"}},
                        {"action": "NAVIGATE_TO", "arguments": {"location": "living room"}},
                        {"action": "PLACE", "arguments": {"object": "cup", "location": "table"}}
                    ]
                }
                """

            # Parse the response
            try:
                action_plan = json.loads(response_text)
                self.get_logger().info(f'Generated action plan: {action_plan["plan"]}')
                return action_plan
            except json.JSONDecodeError:
                self.get_logger().error(f'Failed to parse LLM response: {response_text}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {e}')
            return None

    def execute_action_plan(self, action_plan):
        """Execute the action plan generated by LLM"""
        for action in action_plan['plan']:
            action_type = action['action']
            arguments = action.get('arguments', {})

            self.get_logger().info(f'Executing action: {action_type} with args: {arguments}')

            if action_type == 'NAVIGATE_TO':
                self.execute_navigation(arguments)
            elif action_type == 'PICK_UP':
                self.execute_pickup(arguments)
            elif action_type == 'PLACE':
                self.execute_place(arguments)
            elif action_type == 'SPEAK':
                self.execute_speak(arguments)
            elif action_type == 'GREET':
                self.execute_greet()
            elif action_type == 'WAIT':
                self.execute_wait(arguments)
            elif action_type == 'SCAN_ENVIRONMENT':
                self.execute_scan()
            elif action_type == 'ASK_FOR_HELP':
                self.execute_ask_for_help()

            # Add to action history
            self.action_history.append({
                'action': action,
                'timestamp': time.time(),
                'status': 'completed'
            })

    def execute_navigation(self, arguments):
        """Execute navigation action"""
        location = arguments.get('location', 'unknown')
        self.get_logger().info(f'Navigating to {location}')

        # In practice, this would send navigation goals to Nav2
        # For now, we'll simulate by publishing a goal pose
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()

        # Set goal based on location (simplified)
        if location == 'kitchen':
            goal_msg.pose.position.x = 5.0
            goal_msg.pose.position.y = 2.0
        elif location == 'living room':
            goal_msg.pose.position.x = 2.0
            goal_msg.pose.position.y = 5.0
        else:
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 0.0

        goal_msg.pose.orientation.w = 1.0
        self.navigation_goal_pub.publish(goal_msg.pose)

    def execute_pickup(self, arguments):
        """Execute object pickup action"""
        object_name = arguments.get('object', 'unknown')
        self.get_logger().info(f'Attempting to pick up {object_name}')

        # In practice, this would involve manipulation planning
        cmd_msg = String()
        cmd_msg.data = f'pickup_{object_name}'
        self.robot_cmd_pub.publish(cmd_msg)

    def execute_place(self, arguments):
        """Execute object placement action"""
        object_name = arguments.get('object', 'unknown')
        location = arguments.get('location', 'default')
        self.get_logger().info(f'Placing {object_name} at {location}')

        cmd_msg = String()
        cmd_msg.data = f'place_{object_name}_at_{location}'
        self.robot_cmd_pub.publish(cmd_msg)

    def execute_speak(self, arguments):
        """Execute speech action"""
        text = arguments.get('text', 'Hello')
        self.get_logger().info(f'Speaking: {text}')

        cmd_msg = String()
        cmd_msg.data = f'speak:{text}'
        self.robot_cmd_pub.publish(cmd_msg)

    def execute_greet(self):
        """Execute greeting action"""
        self.get_logger().info('Greeting user')

        cmd_msg = String()
        cmd_msg.data = 'greet'
        self.robot_cmd_pub.publish(cmd_msg)

    def execute_wait(self, arguments):
        """Execute wait action"""
        duration = arguments.get('duration', 1.0)
        self.get_logger().info(f'Waiting for {duration} seconds')

        # In practice, this would use a timer
        time.sleep(min(duration, 5.0))  # Limit wait time for demo

    def execute_scan(self):
        """Execute environment scanning"""
        self.get_logger().info('Scanning environment')

        if self.last_image is not None:
            # Process the last image to identify objects
            self.analyze_environment(self.last_image)
        else:
            self.get_logger().warn('No image available for scanning')

    def execute_ask_for_help(self):
        """Execute help request"""
        self.get_logger().info('Asking for help')

        cmd_msg = String()
        cmd_msg.data = 'help_request'
        self.robot_cmd_pub.publish(cmd_msg)

    def analyze_environment(self, image):
        """Analyze environment from image"""
        # This would typically call a vision LLM or object detection
        # For demonstration, we'll simulate object detection
        detected_objects = [
            {"name": "cup", "confidence": 0.85, "location": {"x": 1.2, "y": 0.8}},
            {"name": "bottle", "confidence": 0.78, "location": {"x": 1.5, "y": 1.2}},
            {"name": "chair", "confidence": 0.92, "location": {"x": 2.0, "y": 0.5}}
        ]

        self.get_logger().info(f'Detected objects: {detected_objects}')

        # Update object locations
        for obj in detected_objects:
            self.object_locations[obj["name"]] = obj["location"]

    def get_recent_observations(self):
        """Get recent environmental observations"""
        return {
            "detected_objects": list(self.object_locations.keys()),
            "last_scan_time": time.time() if self.object_locations else None
        }

def main(args=None):
    rclpy.init(args=args)
    node = LLMCognitivePlanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Prompt Engineering for Robotics

### Effective Prompting Strategies
```python
class PromptEngineer:
    def __init__(self):
        self.robot_capabilities = [
            "navigation to specified locations",
            "object detection and recognition",
            "grasping and manipulation of objects",
            "speech synthesis and recognition",
            "environment mapping and localization",
            "human interaction and social behaviors"
        ]

    def create_task_decomposition_prompt(self, high_level_task):
        """Create prompt for decomposing high-level tasks"""
        prompt = f"""
        Task: {high_level_task}

        You are a task decomposition expert for a humanoid robot. Break down the above task into specific, executable actions that the robot can perform. Consider:

        1. The robot's capabilities: {', '.join(self.robot_capabilities)}
        2. Environmental constraints and safety requirements
        3. The sequence of actions needed to complete the task
        4. Potential failure modes and how to handle them

        Provide the decomposition in the following format:
        - Action sequence with dependencies
        - Required preconditions for each action
        - Expected outcomes for each action
        - Error handling procedures

        Be specific about locations, objects, and actions. Use clear, unambiguous language.
        """
        return prompt

    def create_safety_check_prompt(self, proposed_action):
        """Create prompt for safety validation"""
        safety_prompt = f"""
        Proposed Action: {proposed_action}

        You are a safety validator for a humanoid robot. Analyze the proposed action for potential safety issues:

        1. Collision risks with environment or humans
        2. Mechanical stress on robot components
        3. Environmental hazards
        4. Social appropriateness
        5. Ethical considerations

        For each safety concern, rate the risk level (Low/Medium/High) and suggest mitigation strategies.

        If the action is safe, confirm with "SAFE". If modifications are needed, provide specific recommendations.
        """
        return safety_prompt

    def create_context_awareness_prompt(self, current_context, new_command):
        """Create prompt for context-aware command interpretation"""
        context_prompt = f"""
        Current Context:
        - Robot Location: {current_context.get('location', 'unknown')}
        - Recent Actions: {current_context.get('recent_actions', [])}
        - Detected Objects: {current_context.get('detected_objects', [])}
        - Human Interactions: {current_context.get('human_interactions', [])}

        New Command: {new_command}

        Interpret the new command considering the current context. Consider:
        1. Anaphora resolution (pronouns referring to previously mentioned objects)
        2. Implicit references to locations or objects
        3. Continuation of ongoing tasks
        4. Contextual constraints and preferences

        Provide your interpretation and any clarifying questions if needed.
        """
        return context_prompt

    def create_multi_step_planning_prompt(self, goal, constraints):
        """Create prompt for multi-step planning"""
        planning_prompt = f"""
        Goal: {goal}
        Constraints: {constraints}

        Create a detailed multi-step plan that accounts for:
        1. Temporal dependencies between actions
        2. Resource requirements and availability
        3. Conditional execution based on sensor feedback
        4. Recovery strategies for potential failures
        5. Optimization of efficiency and safety

        Provide the plan in JSON format with:
        - Action sequence with timing estimates
        - Conditional branches and decision points
        - Required resources for each step
        - Success criteria for each action
        - Fallback procedures
        """
        return planning_prompt
```

## Vision-Language Integration

### Multimodal Perception Node
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import base64
import io
from PIL import Image as PILImage

class VisionLanguageIntegrator(Node):
    def __init__(self):
        super().__init__('vision_language_integrator')

        # Publishers
        self.scene_description_pub = self.create_publisher(String, '/scene_description', 10)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # Parameters
        self.declare_parameter('enable_vision_llm', False)
        self.enable_vision_llm = self.get_parameter('enable_vision_llm').value

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Vision LLM client (if available)
        if self.enable_vision_llm:
            import openai
            self.vision_client = openai.OpenAI()

    def image_callback(self, msg):
        """Process image and generate scene description"""
        try:
            # Convert ROS image to PIL Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pil_image = PILImage.fromarray(cv_image)

            # Generate scene description
            if self.enable_vision_llm:
                description = self.describe_scene_with_vision_llm(pil_image)
            else:
                description = self.describe_scene_basic(cv_image)

            # Publish description
            desc_msg = String()
            desc_msg.data = description
            self.scene_description_pub.publish(desc_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def describe_scene_with_vision_llm(self, pil_image):
        """Use vision LLM to describe scene"""
        try:
            # Convert PIL image to base64 for API
            buffer = io.BytesIO()
            pil_image.save(buffer, format='JPEG')
            image_base64 = base64.b64encode(buffer.getvalue()).decode()

            response = self.vision_client.chat.completions.create(
                model="gpt-4-vision-preview",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": "Describe this scene in detail. Identify all objects, their positions, colors, and any spatial relationships. Focus on information that would be relevant for a humanoid robot navigating and interacting with this environment."},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{image_base64}"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=500
            )

            return response.choices[0].message.content

        except Exception as e:
            self.get_logger().error(f'Vision LLM error: {e}')
            return self.describe_scene_basic(np.array(pil_image))

    def describe_scene_basic(self, cv_image):
        """Basic scene description using computer vision"""
        # This would use traditional computer vision techniques
        # For demonstration, return a placeholder
        return "Scene description: Multiple objects detected in the environment. The scene includes furniture, obstacles, and navigable pathways. Further analysis required for detailed object identification and spatial relationships."
```

## Decision Making Framework

### Hierarchical Decision Maker
```python
import enum
from dataclasses import dataclass
from typing import Optional, List
import time

class DecisionPriority(enum.Enum):
    EMERGENCY = 1
    HIGH = 2
    MEDIUM = 3
    LOW = 4

class DecisionType(enum.Enum):
    NAVIGATION = "navigation"
    MANIPULATION = "manipulation"
    SOCIAL = "social"
    MAINTENANCE = "maintenance"

@dataclass
class Decision:
    decision_type: DecisionType
    priority: DecisionPriority
    action: str
    context: dict
    timestamp: float
    confidence: float

class HierarchicalDecisionMaker:
    def __init__(self):
        self.decisions_queue = []
        self.active_decisions = {}
        self.decision_history = []

    def evaluate_decision(self, decision_request):
        """Evaluate a decision request using LLM"""
        # This would call the LLM with appropriate context
        # For demonstration, we'll return a mock decision
        decision = Decision(
            decision_type=DecisionType.NAVIGATION,
            priority=DecisionPriority.MEDIUM,
            action="NAVIGATE_TO_SAFETY",
            context={"location": "charging_station", "reason": "low_battery"},
            timestamp=time.time(),
            confidence=0.85
        )
        return decision

    def prioritize_decisions(self):
        """Sort decisions by priority and urgency"""
        return sorted(self.decisions_queue, key=lambda x: (x.priority.value, -x.confidence))

    def execute_decision(self, decision):
        """Execute a decision with safety checks"""
        # Perform safety validation
        if self.validate_decision_safety(decision):
            self.active_decisions[decision.action] = decision
            self.decision_history.append(decision)
            return True
        return False

    def validate_decision_safety(self, decision):
        """Validate decision safety using multiple checks"""
        # This would integrate with safety systems
        # For now, implement basic checks
        if decision.confidence < 0.5:
            return False  # Low confidence decisions are not safe
        return True

    def cancel_decision(self, action_name):
        """Cancel an active decision"""
        if action_name in self.active_decisions:
            del self.active_decisions[action_name]
```

## Practical Exercise: Cognitive Planning System

### Step 1: Set up LLM integration
1. Configure API access for your chosen LLM
2. Implement the cognitive planning node
3. Test basic command understanding
4. Validate safety and error handling

### Step 2: Implement multimodal integration
1. Connect vision system with LLM
2. Implement scene understanding
3. Test object recognition and description
4. Validate spatial reasoning capabilities

### Step 3: Create decision-making framework
1. Implement hierarchical decision maker
2. Add priority-based execution
3. Test with conflicting objectives
4. Validate safety constraints

## Hands-On Project: Complete Cognitive Planning System

Develop a complete cognitive planning system that includes:
1. LLM-based task decomposition and planning
2. Vision-language integration for scene understanding
3. Hierarchical decision-making framework
4. Safety validation and error handling
5. Integration with robot control systems

## Assignment

1. Implement an LLM-based cognitive planner for your robot
2. Integrate vision-language capabilities for scene understanding
3. Create a decision-making framework with safety validation
4. Test the system with complex, multi-step commands
5. Evaluate the system's performance and safety measures
6. Document the prompt engineering strategies and their effectiveness

## Resources

- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [Anthropic API Documentation](https://docs.anthropic.com/)
- [Robotics and LLMs Research](https://arxiv.org/search/?query=robotics+large+language+model)
- [Prompt Engineering Guide](https://www.promptingguide.ai/)

## Next Week Preview

Next week, we'll conclude the textbook with the capstone project: implementing a complete autonomous humanoid robot system that integrates all the components we've learned about - from ROS 2 communication and simulation to vision-language-action systems. You'll build a system that can understand natural language commands, navigate environments, manipulate objects, and interact naturally with humans.