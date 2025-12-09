---
sidebar_label: 'Week 6: Physics Simulation in Gazebo'
---

# Week 6: Physics Simulation in Gazebo

## Learning Objectives

By the end of this week, you will be able to:
- Set up and configure Gazebo for humanoid robot simulation
- Create realistic physics environments for robot testing
- Implement sensor simulation including LiDAR, cameras, and IMUs
- Integrate Gazebo with ROS 2 for seamless simulation
- Validate robot behaviors in simulated environments

## Overview

Gazebo provides a powerful physics simulation environment that enables testing and validation of robotic systems without requiring physical hardware. This week focuses on creating realistic simulation environments for humanoid robots, including accurate physics modeling, sensor simulation, and integration with ROS 2 for comprehensive testing.

## Gazebo Fundamentals

### Core Components
- **Physics Engine**: Supports ODE, Bullet, Simbody, and DART for realistic physics simulation
- **Sensor Simulation**: Accurate modeling of cameras, LiDAR, IMUs, force/torque sensors
- **Rendering Engine**: High-quality 3D visualization and rendering
- **Plugin Architecture**: Extensible system for custom behaviors and integrations

### Gazebo vs. Real World Considerations
- **Simulation Fidelity**: Balance between accuracy and performance
- **Reality Gap**: Differences between simulation and real-world behavior
- **Transfer Learning**: Techniques to bridge simulation-to-reality gap

## Setting Up Gazebo for Humanoid Robots

### Installation and Configuration
```bash
# Install Gazebo Garden (recommended version)
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control

# Set up environment
source /usr/share/gazebo/setup.sh
```

### Basic Gazebo Launch File
```xml
<?xml version="1.0"?>
<launch>
  <!-- Start Gazebo with empty world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch.py">
    <arg name="world" value="$(find my_robot_gazebo)/worlds/my_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn robot in Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_entity.py"
        args="-entity my_humanoid_robot -topic robot_description -x 0 -y 0 -z 1"
        output="screen"/>
</launch>
```

## Physics Configuration for Humanoid Robots

### Inertial Properties
Accurate inertial properties are crucial for realistic humanoid simulation:

```xml
<!-- Example link with proper inertial properties -->
<link name="left_thigh">
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <inertia ixx="0.05" ixy="0.0" ixz="0.0"
             iyy="0.05" iyz="0.0" izz="0.01"/>
  </inertial>

  <visual>
    <origin xyz="0 0 -0.15"/>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>

  <collision>
    <origin xyz="0 0 -0.15"/>
    <geometry>
      <capsule length="0.3" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

### Joint Configuration for Humanoid Locomotion
```xml
<!-- Example of a humanoid hip joint with proper limits -->
<joint name="left_hip_yaw" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0.05 -0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="3.0"/>
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

## Sensor Simulation

### Camera Sensors
```xml
<gazebo reference="head_camera">
  <sensor type="camera" name="head_camera_sensor">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>head_camera_optical</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Sensors
```xml
<gazebo reference="lidar_link">
  <sensor type="ray" name="humanoid_lidar">
    <pose>0 0 0.5 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensors
```xml
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <namespace>/humanoid_robot</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <initial_orientation_as_reference>false</initial_orientation_as_reference>
    </plugin>
  </sensor>
</gazebo>
```

## Gazebo Controllers for Humanoid Robots

### Joint State Controller
```yaml
# config/humanoid_controllers.yaml
humanoid_joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50

left_leg_controller:
  type: position_controllers/JointTrajectoryController
  joints:
    - left_hip_yaw
    - left_hip_roll
    - left_hip_pitch
    - left_knee
    - left_ankle_pitch
    - left_ankle_roll
  constraints:
    goal_time: 0.6
    stopped_velocity_tolerance: 0.05
    left_hip_yaw:
      trajectory: 0.05
      goal: 0.01
    left_hip_roll:
      trajectory: 0.05
      goal: 0.01

right_leg_controller:
  type: position_controllers/JointTrajectoryController
  joints: [right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle_pitch, right_ankle_roll]
  constraints:
    goal_time: 0.6
    stopped_velocity_tolerance: 0.05
```

## Practical Exercise: Humanoid Robot in Gazebo

### Step 1: Create a Gazebo World
Create a simple world file with obstacles and terrain variations:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="humanoid_test_world">
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add obstacles -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add terrain -->
    <model name="ramp">
      <pose>-2 0 0 0 0.2 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://ramp/meshes/ramp.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://ramp/meshes/ramp.dae</uri>
            </mesh>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Step 2: Launch File for Complete Simulation
```python
# launch/humanoid_simulation.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'my_humanoid_gazebo'

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare(package_name),
                'worlds',
                'humanoid_test_world.world'
            ])
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'humanoid_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity
    ])
```

## Hands-On Project: Complete Humanoid Simulation Environment

Create a complete simulation environment that includes:
1. A humanoid robot model with all necessary sensors
2. A challenging world with obstacles and terrain variations
3. Controllers for all joints to enable basic movement
4. ROS 2 integration for commanding the simulated robot

## Assignment

1. Set up a Gazebo simulation with your humanoid robot model
2. Add at least 3 different sensor types to your robot
3. Create a custom world with obstacles and terrain features
4. Implement basic movement commands to navigate the environment
5. Document the physics parameters used and their effects on robot behavior

## Resources

- [Gazebo Documentation](http://gazebosim.org/)
- [ROS 2 with Gazebo](https://classic.gazebosim.org/tutorials?tut=ros2_overview)
- [Gazebo Plugins](http://gazebosim.org/tutorials/?tut=ros_gzplugins)
- [Physics Tuning Guide](http://gazebosim.org/tutorials?tut=physics_tuning)

## Next Week Preview

Next week, we'll continue with Gazebo simulation, focusing on advanced features and sensor integration. We'll also begin exploring Unity for high-fidelity rendering and visualization of our robotic systems.