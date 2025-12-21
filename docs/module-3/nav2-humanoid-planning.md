---
sidebar_position: 4
---

# Nav2 for Humanoid Planning: Path Planning with Locomotion Constraints

## Introduction to Humanoid Navigation

Navigation for humanoid robots presents unique challenges compared to traditional wheeled robots. Humanoid robots must navigate with bipedal locomotion constraints, including balance maintenance, step planning, and dynamic stability. Nav2, the standard navigation stack for ROS 2, can be configured with humanoid-specific constraints to enable effective navigation for bipedal robots.

## Key Differences from Wheeled Robot Navigation

### Locomotion Constraints
- **Balance Requirements**: Maintaining center of mass within support polygon
- **Step Planning**: Discrete foot placement rather than continuous motion
- **Dynamic Stability**: Managing dynamic walking patterns
- **Terrain Requirements**: Need for stable footholds and traversable surfaces

### Navigation Challenges
- **Limited Step Height**: Cannot step over large obstacles
- **Footprint Variations**: Changing support polygon during movement
- **Turning Radius**: Different turning capabilities than wheeled robots
- **Slope Limitations**: Maximum incline angles for stable walking

## Nav2 Architecture for Humanoid Robots

### Core Components
1. **Global Planner**: Path planning considering humanoid constraints
2. **Local Planner**: Footstep planning and dynamic obstacle avoidance
3. **Controller**: Humanoid-specific motion control
4. **Costmap**: Humanoid-aware obstacle representation
5. **Recovery Behaviors**: Humanoid-specific recovery actions

### Humanoid-Specific Modifications

```yaml
# Nav2 configuration for humanoid robot
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: false
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "navigate_w_replanning_and_humanoid_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_humanoid_recovery.xml"
    navigate_through_poses_xxx: "humanoid_navigate_through_poses_w_replanning_and_recovery.xml"
    navigate_to_pose_xxx: "humanoid_navigate_to_pose_w_replanning_and_recovery.xml"

    # Behavior Tree Nodes
    humanoid_clear_costmap_recovery:
      plugin: "humanoid_recoveries/ClearHumanoidCostmap"
    humanoid_spin:
      plugin: "humanoid_recoveries/Spin"
    humanoid_backup:
      plugin: "humanoid_recoveries/Backup"
    humanoid_drive_on_heading:
      plugin: "humanoid_recoveries/DriveOnHeading"
    humanoid_wait:
      plugin: "humanoid_recoveries/Wait"
```

## Humanoid-Specific Costmap Configuration

The costmap for humanoid robots needs to account for locomotion constraints:

```yaml
# Costmap configuration for humanoid navigation
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: false

      # Humanoid-specific parameters
      robot_radius: 0.4  # Approximate radius for humanoid footprint
      footprint_padding: 0.05
      resolution: 0.05  # Finer resolution for step planning

      # Locomotion constraints
      max_step_height: 0.15  # Maximum step height in meters
      max_step_width: 0.4   # Maximum step width in meters
      min_turn_radius: 0.5  # Minimum turning radius

      plugins: [
        "static_layer",
        "obstacle_layer",
        "voxel_layer",
        "inflation_layer"
      ]

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 5
      height: 5
      resolution: 0.05

      # Humanoid-specific local parameters
      robot_radius: 0.4
      max_step_height: 0.15
      max_step_width: 0.4
      min_turn_radius: 0.5

      plugins: [
        "obstacle_layer",
        "voxel_layer",
        "inflation_layer"
      ]
```

## Footstep Planning

### Discrete Step Planning
Humanoid navigation requires planning discrete foot placements:

```cpp
// Example footstep planner for humanoid robots
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class HumanoidFootstepPlanner : public nav2_core::GlobalPlanner
{
public:
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        // Configure footstep planning parameters
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // Initialize humanoid-specific parameters
        max_step_height_ = node_->declare_parameter(name + ".max_step_height", 0.15);
        max_step_width_ = node_->declare_parameter(name + ".max_step_width", 0.4);
        max_pitch_angle_ = node_->declare_parameter(name + ".max_pitch_angle", 0.3);
        max_roll_angle_ = node_->declare_parameter(name + ".max_roll_angle", 0.2);
    }

    void cleanup() override
    {
        RCLCPP_INFO(
            node_->get_logger(), "Cleaning up planner: %s", name_.c_str());
    }

    void activate() override
    {
        RCLCPP_INFO(
            node_->get_logger(), "Activating planner: %s", name_.c_str());
    }

    void deactivate() override
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating planner: %s", name_.c_str());
    }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override
    {
        nav_msgs::msg::Path path;

        // Check if goal is reachable given humanoid constraints
        if (!isGoalReachable(start, goal)) {
            return path; // Return empty path
        }

        // Perform footstep planning considering humanoid constraints
        path = planFootsteps(start, goal);

        return path;
    }

private:
    bool isGoalReachable(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal)
    {
        // Check if the path is traversable considering step height constraints
        // This is a simplified check - real implementation would be more complex
        double height_diff = std::abs(goal.pose.position.z - start.pose.position.z);
        return height_diff <= max_step_height_;
    }

    nav_msgs::msg::Path planFootsteps(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->now();

        // Implement footstep planning algorithm
        // This would involve finding valid footholds and planning discrete steps
        // considering balance and stability constraints

        // For simplicity, we'll create a straight-line path
        // In practice, this would involve complex footstep planning
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        // Add start pose
        pose.pose = start.pose;
        path.poses.push_back(pose);

        // Add intermediate steps (simplified)
        double step_size = 0.3; // Average step size for humanoid
        double dist = std::sqrt(
            std::pow(goal.pose.position.x - start.pose.position.x, 2) +
            std::pow(goal.pose.position.y - start.pose.position.y, 2)
        );

        int num_steps = static_cast<int>(dist / step_size);

        for (int i = 1; i < num_steps; ++i) {
            double ratio = static_cast<double>(i) / num_steps;
            pose.pose.position.x =
                start.pose.position.x + ratio * (goal.pose.position.x - start.pose.position.x);
            pose.pose.position.y =
                start.pose.position.y + ratio * (goal.pose.position.y - start.pose.position.y);
            pose.pose.position.z =
                start.pose.position.z + ratio * (goal.pose.position.z - start.pose.position.z);

            // Add orientation interpolation
            pose.pose.orientation = interpolateOrientation(
                start.pose.orientation, goal.pose.orientation, ratio);

            path.poses.push_back(pose);
        }

        // Add goal pose
        pose.pose = goal.pose;
        path.poses.push_back(pose);

        return path;
    }

    geometry_msgs::msg::Quaternion interpolateOrientation(
        const geometry_msgs::msg::Quaternion& start,
        const geometry_msgs::msg::Quaternion& end,
        double t)
    {
        // Simple linear interpolation for demonstration
        // In practice, use spherical linear interpolation (SLERP)
        geometry_msgs::msg::Quaternion result;
        result.x = start.x + t * (end.x - start.x);
        result.y = start.y + t * (end.y - start.y);
        result.z = start.z + t * (end.z - start.z);
        result.w = start.w + t * (end.w - start.w);

        // Normalize quaternion
        double norm = std::sqrt(result.x*result.x + result.y*result.y +
                               result.z*result.z + result.w*result.w);
        if (norm > 0.0) {
            result.x /= norm;
            result.y /= norm;
            result.z /= norm;
            result.w /= norm;
        }

        return result;
    }

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_costmap_2d::Costmap2D* costmap_;
    std::string global_frame_;

    // Humanoid-specific parameters
    double max_step_height_;
    double max_step_width_;
    double max_pitch_angle_;
    double max_roll_angle_;
};
```

## Balance and Stability Considerations

### Center of Mass Management
Humanoid navigation must consider balance constraints:

```yaml
# Balance constraint configuration
balance_constraints:
  center_of_mass_height: 0.8  # Height of CoM in meters
  max_com_deviation: 0.1      # Maximum CoM deviation from support polygon
  support_polygon_margin: 0.05 # Safety margin for support polygon

# Footprint definition for balance
foot_print:
  # Left foot positions (x, y coordinates relative to base_link)
  left_foot:
    - [0.1,  0.1]   # front left
    - [0.1, -0.1]   # front right
    - [-0.1, -0.1]  # back right
    - [-0.1, 0.1]   # back left

  # Right foot positions
  right_foot:
    - [0.1,  0.1]
    - [0.1, -0.1]
    - [-0.1, -0.1]
    - [-0.1, 0.1]
```

## Integration with Isaac ROS Perception

### Perception-Driven Navigation
Combining Isaac ROS perception with Nav2 for intelligent navigation:

```python
# Python example of perception-integrated navigation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np

class PerceptionIntegratedNavigator(Node):
    def __init__(self):
        super().__init__('perception_integrated_navigator')

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Perception data subscribers
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/perception/lidar_points',
            self.pointcloud_callback,
            10
        )

        # Timer for navigation updates
        self.nav_timer = self.create_timer(1.0, self.navigation_callback)

        # Navigation state
        self.current_goal = None
        self.perception_data = None

    def pointcloud_callback(self, msg):
        # Process point cloud data from Isaac ROS perception
        # Update costmaps based on perceived obstacles
        self.perception_data = msg
        self.update_navigation_plan()

    def navigation_callback(self):
        # Check if we need to update navigation based on perception data
        if self.perception_data and self.current_goal:
            # Replan path considering new perception data
            self.replan_with_perception()

    def navigate_with_perception(self, goal_pose):
        # Navigate to goal while continuously updating based on perception
        self.current_goal = goal_pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Wait for server
        self.nav_client.wait_for_server()

        # Send goal
        future = self.nav_client.send_goal_async(goal_msg)
        return future

    def update_navigation_plan(self):
        # Update navigation plan based on perception data
        # This could involve replanning or adjusting the current plan
        pass

    def replan_with_perception(self):
        # Replan the navigation path based on updated perception data
        # This would consider obstacles detected by Isaac ROS perception
        pass
```

## Configuration Examples

### Complete Nav2 Configuration for Humanoid Robot

```yaml
# Complete Nav2 configuration with humanoid constraints
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
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
    save_pose_delay: 0.2
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.2
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: false
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_nav_through_poses_bt_xml: "navigate_w_replanning_and_humanoid_recovery.xml"
    default_nav_to_pose_bt_xml: "navigate_w_replanning_and_humanoid_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "humanoid_controllers/FootstepController"
      speed_ratio: 0.25
      step_size: 0.3
      max_step_height: 0.15
      balance_margin: 0.05

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      robot_radius: 0.4
      plugins: ["obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
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
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: true
      robot_radius: 0.4
      resolution: 0.05
      track_unknown_space: false
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
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
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "humanoid_planners/HumanoidGridBasedPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
      max_iterations: 10000
      max_on_approach_iterations: 1000
      humanoid_step_constraints:
        max_step_height: 0.15
        max_step_width: 0.4
        min_turn_radius: 0.5
        balance_margin: 0.05
```

## Testing and Validation

### Simulation Testing
Before deploying on real humanoid robots, test navigation in simulation:

1. **Isaac Sim Integration**: Test navigation in photorealistic environments
2. **Gazebo Simulation**: Validate with physics-accurate simulation
3. **Unity Environments**: Test in diverse virtual worlds

### Performance Metrics
- **Path Optimality**: Compare planned vs optimal paths
- **Balance Maintenance**: Monitor center of mass during navigation
- **Obstacle Avoidance**: Validate safe navigation around obstacles
- **Step Planning**: Verify feasible footstep sequences

## Troubleshooting Common Issues

### Navigation Failures
- **Path Planning Failures**: Check costmap inflation and resolution
- **Balance Issues**: Verify center of mass constraints
- **Step Planning Problems**: Adjust step size and height parameters
- **Localization Drift**: Improve sensor fusion and calibration

### Performance Optimization
- **Computation Time**: Optimize costmap resolution and update rates
- **Memory Usage**: Monitor and optimize data structures
- **Real-time Performance**: Prioritize critical navigation tasks

## Hands-on Exercise

### Exercise 1: Humanoid Nav2 Configuration
Configure Nav2 for a humanoid robot simulation:

1. Set up the Nav2 stack with humanoid constraints
2. Configure costmaps for step planning
3. Implement basic footstep planning
4. Test navigation in a simple environment
5. Analyze path optimality and balance maintenance

### Exercise 2: Perception-Integrated Navigation
Combine Isaac ROS perception with Nav2:

1. Set up Isaac ROS perception pipeline
2. Integrate perception data with Nav2 costmaps
3. Test dynamic obstacle avoidance
4. Evaluate navigation performance with real-time perception updates
5. Compare results with perception-free navigation

## Summary

Nav2 for humanoid robots requires special considerations for bipedal locomotion, balance maintenance, and discrete step planning. By configuring Nav2 with appropriate constraints and integrating with Isaac ROS perception, you can create robust navigation systems for humanoid robots.

The key to successful humanoid navigation lies in properly accounting for balance constraints, step planning limitations, and dynamic stability requirements. With the right configuration and integration, Nav2 provides a powerful foundation for humanoid robot navigation.

This completes Module 3, where you've learned about Isaac Sim for synthetic data generation, Isaac ROS for hardware-accelerated perception, and Nav2 configuration for humanoid-specific navigation. These technologies form the core of the AI-robot brain for humanoid robotics applications.