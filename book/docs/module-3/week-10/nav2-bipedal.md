---
sidebar_label: 'Week 10: Nav2 for Bipedal Movement and Path Planning'
---

# Week 10: Nav2 for Bipedal Movement and Path Planning

## Learning Objectives

By the end of this week, you will be able to:
- Configure Nav2 specifically for humanoid robot navigation
- Implement bipedal-specific path planning algorithms
- Adapt costmaps for humanoid locomotion constraints
- Develop footstep planning integration with Nav2
- Optimize navigation parameters for stable bipedal movement
- Evaluate and validate humanoid navigation performance

## Overview

Navigation 2 (Nav2) is the state-of-the-art navigation stack for ROS 2, but requires specific adaptations for humanoid robots. This week focuses on configuring Nav2 for bipedal locomotion, considering the unique challenges of two-legged movement including balance constraints, step planning, and human-aware navigation behaviors.

## Nav2 Architecture for Humanoid Robots

### Standard Nav2 Components
- **Global Planner**: Generates optimal path from start to goal
- **Local Planner**: Executes path while avoiding obstacles
- **Controller**: Translates path to robot commands
- **Recovery Behaviors**: Handles navigation failures
- **Costmap 2D**: Represents obstacles and navigation areas

### Humanoid-Specific Adaptations
- **Footstep Planner**: Generates stable stepping sequences
- **Balance Controller**: Maintains center of mass during movement
- **Step Height Constraints**: Limits step height for stable locomotion
- **Gait Integration**: Coordinates with walking pattern generators
- **Stability Constraints**: Ensures center of mass remains within support polygon

## Nav2 Configuration for Humanoid Robots

### Basic Nav2 Launch File for Humanoid
```xml
<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="namespace" default=""/>
  <arg name="use_sim_time" default="true"/>
  <arg name="autostart" default="true"/>
  <arg name="params_file" default="$(find-pkg-share my_humanoid_nav2_config)/params/humanoid_nav2_params.yaml"/>
  <arg name="bt_xml_file" default="$(find-pkg-share nav2_bt_navigator)/behavior_trees/multi_room.xml"/>
  <arg name="map" default="$(find-pkg-share my_humanoid_nav2_config)/maps/humanoid_world.yaml"/>

  <!-- Map Server -->
  <node pkg="nav2_map_server" exec="map_server" name="map_server" output="screen">
    <param name="yaml_filename" value="$(var map)"/>
    <param name="topic" value="map"/>
    <param name="frame_id" value="map"/>
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Local Costmap -->
  <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="local_costmap" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="robot_base_frame" value="base_link"/>
    <param name="global_frame" value="odom"/>
    <param name="rolling_window" value="true"/>
    <param name="width" value="10"/>
    <param name="height" value="10"/>
    <param name="resolution" value="0.05"/>
    <param name="footprint" value="[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"/>
    <param name="plugins" value="[
      {name: inflation_layer, type: nav2_costmap_2d::InflationLayer},
      {name: obstacle_layer, type: nav2_costmap_2d::ObstacleLayer},
      {name: voxel_layer, type: nav2_costmap_2d::VoxelLayer}]"/>
  </node>

  <!-- Global Costmap -->
  <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="global_costmap" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="robot_base_frame" value="base_link"/>
    <param name="global_frame" value="map"/>
    <param name="robot_radius" value="0.4"/>
    <param name="plugins" value="[
      {name: inflation_layer, type: nav2_costmap_2d::InflationLayer},
      {name: obstacle_layer, type: nav2_costmap_2d::ObstacleLayer}]"/>
  </node>

  <!-- Nav2 Stack -->
  <node pkg="nav2_planner" exec="planner_server" name="planner_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="planner_plugins" value="GridBased"/>
    <param name="GridBased.name" value="GridBased"/>
    <param name="GridBased.type" value="nav2_navfn_planner/NavfnPlanner"/>
  </node>

  <node pkg="nav2_controller" exec="controller_server" name="controller_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="controller_frequency" value="20.0"/>
    <param name="min_x_velocity_threshold" value="0.001"/>
    <param name="min_y_velocity_threshold" value="0.001"/>
    <param name="min_theta_velocity_threshold" value="0.001"/>
    <param name="progress_checker_plugins" value="progress_checker"/>
    <param name="goal_checker_plugins" value="general_goal_checker"/>
    <param name="controller_plugins" value="FollowPath"/>
    <param name="FollowPath.name" value="FollowPath"/>
    <param name="FollowPath.type" value="nav2_mppi_controller::MPPIController"/>
  </node>

  <node pkg="nav2_recoveries" exec="recoveries_server" name="recoveries_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="bt_xml_filename" value="$(var bt_xml_file)"/>
    <param name="default_server_timeout" value="20"/>
    <param name="goal_node_tolerance" value="0.25"/>
  </node>

  <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager_navigation" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="autostart" value="$(var autostart)"/>
    <param name="node_names" value="[map_server, local_costmap, global_costmap, planner_server, controller_server, recoveries_server, bt_navigator]"/>
  </node>
</launch>
```

### Humanoid-Specific Parameters Configuration
```yaml
# params/humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

amcl_map_client:
  ros__parameters:
    use_sim_time: True

amcl_rclcpp_node:
  ros__parameters:
    use_sim_time: True

bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    # Humanoid-specific behavior tree modifications
    goal_node_tolerance: 0.3  # Increased for humanoid step precision
    goal_function: "simple_goal_checker"
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    # Custom behavior tree for humanoid navigation
    bt_xml_filename: "humanoid_behavior_tree.xml"

bt_navigator_rclcpp_node:
  ros__parameters:
    use_sim_time: True

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.01  # Lower for precise humanoid movement
    min_y_velocity_threshold: 0.01
    min_theta_velocity_threshold: 0.01
    # Humanoid-specific progress checker
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Humanoid FollowPath controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 30
      model_dt: 0.05
      batch_size: 1000
      vx_std: 0.2
      vy_std: 0.1
      wz_std: 0.3
      vx_max: 0.3  # Reduced for humanoid stability
      vx_min: -0.2
      vy_max: 0.1
      wz_max: 0.3
      trajectory_visualization: True
      # Humanoid-specific cost parameters
      goal_dist_cost_param: 24.0
      goal_angle_cost_param: 2.0
      path_cost_param: 6.0
      path_angle_cost_param: 0.05
      obstacle_cost_param: 3.0
      constraint_cost_param: 10.0

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_footprint
      use_sim_time: True
      rolling_window: true
      width: 8  # Humanoid-specific window size
      height: 8
      resolution: 0.05  # Higher resolution for precise footstep planning
      robot_radius: 0.35  # Humanoid robot radius
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # Humanoid-specific inflation
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: pointcloud
        pointcloud:
          topic: /humanoid_pointcloud
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
          min_obstacle_height: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_footprint
      use_sim_time: True
      robot_radius: 0.4  # Humanoid robot radius
      resolution: 0.05  # Higher resolution for detailed planning
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 2.0
        inflation_radius: 0.6
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5  # Humanoid-specific tolerance
      use_astar: false
      allow_unknown: true

smoother_server:
  ros__parameters:
    smoother_plugins: ["simple_smoother"]
    simple_smoother:
      plugin: "nav2_smoother::SimpleSmoother"
      tolerance: 1.0e-10
      max_its: 1000
      do_refinement: True

behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
    spin:
      plugin: "nav2_behaviors/Spin"
      spin_dist: 1.57  # 90 degrees for humanoid turning
    backup:
      plugin: "nav2_behaviors/BackUp"
      backup_dist: 0.15  # Shorter backup for humanoid
      backup_speed: 0.05
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
      drive_on_heading_dist: 0.5
      drive_on_heading_additive_dist: 0.0
      drive_on_heading_forward_sampling_dist: 0.5
      drive_on_heading_lateral_sampling_dist: 0.5
    wait:
      plugin: "nav2_behaviors/Wait"
      wait_duration: 1s
```

## Footstep Planning Integration

### Humanoid Footstep Planner Node
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.spatial.transform import Rotation as R

class HumanoidFootstepPlanner(Node):
    def __init__(self):
        super().__init__('humanoid_footstep_planner')

        # Publishers
        self.footstep_pub = self.create_publisher(Path, '/footstep_plan', 10)
        self.footstep_viz_pub = self.create_publisher(MarkerArray, '/footstep_visualization', 10)

        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            PoseStamped, '/humanoid/pose', self.odom_callback, 10)

        # Footstep planning parameters
        self.step_length = 0.3  # meters
        self.step_width = 0.15  # meters
        self.step_height = 0.05  # meters (for stepping over small obstacles)
        self.max_step_rotation = 0.2  # radians
        self.support_polygon_margin = 0.1  # safety margin

        # Robot state
        self.current_pose = None
        self.left_foot_pose = None
        self.right_foot_pose = None
        self.support_foot = 'left'  # Start with left foot as support

    def path_callback(self, msg):
        """Process global path and generate footstep plan"""
        if not self.current_pose:
            return

        # Generate footstep plan from global path
        footstep_plan = self.generate_footstep_plan(msg.poses)

        # Publish footstep plan
        self.footstep_pub.publish(footstep_plan)

        # Publish visualization
        self.publish_footstep_visualization(footstep_plan)

    def generate_footstep_plan(self, global_path):
        """Generate stable footstep sequence from global path"""
        footstep_path = Path()
        footstep_path.header.frame_id = 'map'
        footstep_path.header.stamp = self.get_clock().now().to_msg()

        if len(global_path) < 2:
            return footstep_path

        # Start with current foot positions
        footsteps = self.initialize_footsteps()

        # Generate footsteps along the path
        path_idx = 0
        current_support_foot = self.support_foot

        for i in range(len(global_path) - 1):
            # Calculate direction vector
            start_pose = global_path[i].pose.position
            end_pose = global_path[i + 1].pose.position
            dx = end_pose.x - start_pose.x
            dy = end_pose.y - start_pose.y
            distance = np.sqrt(dx*dx + dy*dy)

            # Calculate required steps based on step length
            num_steps = int(distance / self.step_length) + 1

            for step in range(num_steps):
                # Calculate step position
                t = step / num_steps
                step_x = start_pose.x + t * dx
                step_y = start_pose.y + t * dy

                # Calculate step orientation (follow path direction)
                if i < len(global_path) - 1:
                    next_start = global_path[i].pose.position
                    next_end = global_path[i + 1].pose.position
                    step_yaw = np.arctan2(next_end.y - next_start.y, next_end.x - next_start.x)
                else:
                    step_yaw = 0  # Default orientation

                # Alternate feet for stability
                if current_support_foot == 'left':
                    swing_foot_pose = self.calculate_swing_foot_pose(
                        step_x, step_y, step_yaw, 'right')
                    current_support_foot = 'right'
                else:
                    swing_foot_pose = self.calculate_swing_foot_pose(
                        step_x, step_y, step_yaw, 'left')
                    current_support_foot = 'left'

                # Add footstep to plan
                footstep_pose = PoseStamped()
                footstep_pose.header.frame_id = 'map'
                footstep_pose.header.stamp = self.get_clock().now().to_msg()
                footstep_pose.pose.position = swing_foot_pose.position
                footstep_pose.pose.orientation = swing_foot_pose.orientation

                footsteps.append(footstep_pose)

        footstep_path.poses = footsteps
        return footstep_path

    def initialize_footsteps(self):
        """Initialize footsteps based on current robot state"""
        footsteps = []

        # Add initial foot positions if available
        if self.left_foot_pose and self.right_foot_pose:
            left_pose = PoseStamped()
            left_pose.header.frame_id = 'map'
            left_pose.pose = self.left_foot_pose
            footsteps.append(left_pose)

            right_pose = PoseStamped()
            right_pose.header.frame_id = 'map'
            right_pose.pose = self.right_foot_pose
            footsteps.append(right_pose)

        return footsteps

    def calculate_swing_foot_pose(self, target_x, target_y, target_yaw, foot_type):
        """Calculate pose for swing foot based on target and support foot"""
        from geometry_msgs.msg import Pose

        # Create pose for the swing foot
        pose = Pose()

        # Position with step width offset for stability
        if foot_type == 'left':
            # Offset to the left relative to movement direction
            offset_x = -self.step_width/2 * np.sin(target_yaw)
            offset_y = self.step_width/2 * np.cos(target_yaw)
        else:
            # Offset to the right relative to movement direction
            offset_x = self.step_width/2 * np.sin(target_yaw)
            offset_y = -self.step_width/2 * np.cos(target_yaw)

        pose.position.x = target_x + offset_x
        pose.position.y = target_y + offset_y
        pose.position.z = 0.0  # Ground level

        # Set orientation
        r = R.from_euler('z', target_yaw)
        quat = r.as_quat()
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    def publish_footstep_visualization(self, footstep_path):
        """Publish visualization markers for footsteps"""
        marker_array = MarkerArray()

        for i, footstep in enumerate(footstep_path.poses):
            # Create marker for this footstep
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'footsteps'
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Position and size
            marker.pose = footstep.pose
            marker.scale.x = 0.1  # Foot size
            marker.scale.y = 0.06  # Foot size
            marker.scale.z = 0.01  # Height

            # Color based on foot type (alternating)
            if i % 2 == 0:
                marker.color.r = 1.0  # Left foot - red
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0  # Right foot - blue
                marker.color.g = 0.0
                marker.color.b = 1.0

            marker.color.a = 0.8
            marker_array.markers.append(marker)

        self.footstep_viz_pub.publish(marker_array)

    def odom_callback(self, msg):
        """Update robot pose"""
        self.current_pose = msg.pose
```

## Balance and Stability Controllers

### Center of Mass Controller
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import numpy as np

class HumanoidBalanceController(Node):
    def __init__(self):
        super().__init__('humanoid_balance_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.com_offset_pub = self.create_publisher(Float64MultiArray, '/com_offset', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.footstep_sub = self.create_subscription(
            Twist, '/footstep_cmd', self.footstep_callback, 10)

        # Balance control parameters
        self.com_reference = np.array([0.0, 0.0, 0.8])  # Reference CoM position
        self.support_polygon = []  # Current support polygon vertices
        self.balance_kp = 2.0  # Proportional gain for balance
        self.balance_kd = 1.0  # Derivative gain for balance

        # Robot parameters
        self.robot_height = 1.0  # Approximate robot height
        self.step_width = 0.3    # Distance between feet

        # Internal state
        self.current_com = np.array([0.0, 0.0, 0.8])
        self.com_velocity = np.array([0.0, 0.0, 0.0])
        self.imu_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # w, x, y, z
        self.imu_angular_velocity = np.array([0.0, 0.0, 0.0])

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        # Update orientation
        self.imu_orientation = np.array([
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z
        ])

        # Update angular velocity
        self.imu_angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Estimate CoM position (simplified)
        self.estimate_com_position()

        # Update support polygon based on foot positions
        self.update_support_polygon()

    def estimate_com_position(self):
        """Estimate center of mass position from IMU and kinematics"""
        # This is a simplified estimation
        # In practice, use full kinematic model and force sensors
        self.current_com[0] += self.com_velocity[0] * 0.05  # 20Hz update
        self.current_com[1] += self.com_velocity[1] * 0.05
        self.current_com[2] = self.robot_height * 0.8  # Approximate CoM height

    def update_support_polygon(self):
        """Update support polygon based on current foot positions"""
        # Simplified: assume feet are at fixed positions relative to base
        # In practice, use forward kinematics with joint angles
        self.support_polygon = [
            np.array([-0.1, -self.step_width/2, 0.0]),  # Left foot
            np.array([-0.1, self.step_width/2, 0.0]),   # Right foot
            np.array([0.1, self.step_width/2, 0.0]),
            np.array([0.1, -self.step_width/2, 0.0])
        ]

    def footstep_callback(self, msg):
        """Process footstep commands and adjust balance"""
        # Calculate balance correction based on desired movement
        balance_correction = self.calculate_balance_correction(msg)

        # Apply balance correction to velocity command
        corrected_cmd = self.apply_balance_correction(msg, balance_correction)

        # Publish corrected command
        self.cmd_vel_pub.publish(corrected_cmd)

    def calculate_balance_correction(self, desired_cmd):
        """Calculate balance correction based on CoM position"""
        # Calculate error from reference CoM
        com_error = self.current_com[:2] - self.com_reference[:2]

        # Calculate if CoM is outside support polygon
        if not self.is_com_stable():
            # Generate correction to bring CoM back to safe region
            correction = -self.balance_kp * com_error - self.balance_kd * self.com_velocity[:2]
        else:
            correction = np.array([0.0, 0.0])

        return correction

    def is_com_stable(self):
        """Check if center of mass is within support polygon"""
        # Simplified stability check
        # In practice, use proper polygon-point inclusion test
        com_x, com_y = self.current_com[0], self.current_com[1]

        # Check if CoM is roughly within foot positions
        if (abs(com_y) < self.step_width/2 + 0.1 and  # Within foot width
            -0.2 < com_x < 0.2):  # Within foot length
            return True
        return False

    def apply_balance_correction(self, original_cmd, correction):
        """Apply balance correction to original command"""
        corrected_cmd = Twist()

        # Apply correction to linear velocity
        corrected_cmd.linear.x = min(
            max(original_cmd.linear.x + correction[0], -0.3), 0.3)
        corrected_cmd.linear.y = min(
            max(original_cmd.linear.y + correction[1], -0.1), 0.1)
        corrected_cmd.linear.z = original_cmd.linear.z

        # Apply to angular velocity
        corrected_cmd.angular = original_cmd.angular

        return corrected_cmd

    def publish_balance_state(self):
        """Publish balance state information"""
        balance_state = Float64MultiArray()
        balance_state.data = [
            float(self.current_com[0]), float(self.current_com[1]), float(self.current_com[2]),
            float(self.com_velocity[0]), float(self.com_velocity[1]), float(self.com_velocity[2]),
            1.0 if self.is_com_stable() else 0.0  # Stability flag
        ]
        self.com_offset_pub.publish(balance_state)
```

## Humanoid-Specific Navigation Behaviors

### Behavior Tree for Humanoid Navigation
```xml
<?xml version="1.0" encoding="UTF-8"?>
<root BTCPP_format="4">
    <BehaviorTree ID="HumanoidMainTree">
        <ReactiveSequence>
            <GoalUpdated/>
            <PipelineSequence>
                <Sequence>
                    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                    <Fallback>
                        <GenerateFootstepPlan path="{path}" footstep_plan="{footstep_plan}"/>
                        <RecoveryNode number_of_retries="2" slow_recoveries="false">
                            <Sequence>
                                <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
                                <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
                            </Sequence>
                            <ReactiveFallback>
                                <GoalUpdated/>
                                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                            </ReactiveFallback>
                        </RecoveryNode>
                    </Fallback>
                </Sequence>
                <Sequence>
                    <FollowPath path="{path}" controller_id="FollowPath"/>
                    <IsGoalReached goal="{goal}" output="is_goal_reached"/>
                </Sequence>
            </PipelineSequence>
        </ReactiveSequence>
    </BehaviorTree>

    <BehaviorTree ID="HumanoidRecoveryTree">
        <ReactiveFallback>
            <RecoveryNode number_of_retries="4" slow_recoveries="true">
                <Spin spin_dist="1.57"/>
                <Wait wait_duration="5"/>
            </RecoveryNode>
            <RecoveryNode number_of_retries="4" slow_recoveries="true">
                <Backup backup_dist="0.15" backup_speed="0.05"/>
                <Wait wait_duration="5"/>
            </RecoveryNode>
            <GlobalRecovery recovery_node_names="Wait"/>
        </ReactiveFallback>
    </BehaviorTree>
</root>
```

## Practical Exercise: Complete Humanoid Navigation System

### Step 1: Configure Nav2 for Humanoid Parameters
1. Set up costmap parameters for humanoid size and constraints
2. Configure controller for stable bipedal movement
3. Adjust planner parameters for step-aware navigation
4. Set up recovery behaviors for humanoid-specific failures

### Step 2: Implement Footstep Planning
1. Create footstep planner node
2. Integrate with Nav2's path following
3. Add visualization for footstep plans
4. Test with simulated humanoid robot

### Step 3: Balance Control Integration
1. Implement CoM tracking and control
2. Add stability checks and corrections
3. Integrate with navigation commands
4. Test balance during navigation

## Hands-On Project: Humanoid Navigation Stack

Develop a complete navigation system that includes:
1. Nav2 configuration specifically for humanoid robots
2. Footstep planning integration
3. Balance and stability control
4. Humanoid-aware behavior trees
5. Comprehensive testing in simulation

## Assignment

1. Configure Nav2 with humanoid-specific parameters
2. Implement a footstep planner that integrates with Nav2
3. Create a balance controller for stable locomotion
4. Test the complete navigation system in simulation
5. Document the configuration parameters and their effects on navigation performance
6. Evaluate navigation success rates and stability metrics

## Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [Humanoid Navigation Papers](https://arxiv.org/search/?query=humanoid+navigation)
- [Footstep Planning Libraries](https://github.com/ahornung/humanoid_navigation)
- [Balance Control Techniques](https://www.cs.cmu.edu/~cga/dyn3d/)

## Next Week Preview

Next week, we'll transition to Module 4, focusing on Vision-Language-Action (VLA) systems. You'll learn to implement voice recognition, LLM integration, and create cognitive planning systems that enable humanoid robots to understand and respond to complex human commands.