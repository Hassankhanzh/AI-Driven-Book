# Data Model: 3-isaac-ai-brain

**Feature**: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)
**Date**: 2025-12-21

## Key Entities

### Isaac Sim Environment Configuration
- **Name**: string (required) - Unique identifier for the simulation environment
- **Description**: string - Brief description of the environment purpose
- **Lighting Settings**: object - Configuration for realistic lighting
  - ambient_intensity: float
  - directional_light: object
    - intensity: float
    - direction: object (x, y, z coordinates)
    - color: object (r, g, b values)
- **Material Properties**: array of objects
  - name: string
  - albedo: object (r, g, b values)
  - roughness: float
  - metallic: float
  - normal_map_strength: float
- **Synthetic Dataset Parameters**: object
  - output_format: enum (RGB, Depth, Semantic Segmentation, etc.)
  - image_resolution: object (width, height)
  - annotation_format: enum (COCO, YOLO, Pascal VOC, etc.)
  - diversity_settings: object
    - lighting_variations: int
    - object_poses: int
    - background_variations: int

### Isaac ROS Perception Pipeline
- **Pipeline Name**: string (required) - Identifier for the perception pipeline
- **VSLAM Configuration**: object
  - algorithm: enum (ORB-SLAM, RTABMAP, etc.)
  - camera_parameters: object
    - focal_length: object (fx, fy)
    - principal_point: object (cx, cy)
    - distortion_coefficients: array of floats
  - gpu_acceleration: boolean
  - max_features: int
  - tracking_threshold: float
- **Sensor Integration**: object
  - camera_topics: array of strings
  - lidar_topic: string (optional)
  - imu_topic: string (optional)
- **Output Formats**: array of strings (e.g., sensor_msgs/Image, geometry_msgs/PoseStamped)

### Nav2 Humanoid Planner Configuration
- **Planner Name**: string (required) - Identifier for the navigation planner
- **Locomotion Constraints**: object
  - max_step_height: float
  - max_step_width: float
  - min_turn_radius: float
  - max_pitch_angle: float
  - max_roll_angle: float
- **Path Planning Parameters**: object
  - global_planner: string (e.g., NavFn, GlobalPlanner, THETA*)
  - local_planner: string (e.g., DWA, TEB, MPC)
  - costmap_resolution: float
  - robot_radius: float
  - inflation_radius: float
- **Humanoid-Specific Settings**: object
  - foot_print: array of objects (x, y coordinates for foot positions)
  - center_of_mass_height: float
  - balance_constraints: object
    - max_com_deviation: float
    - support_polygon_margin: float

### Training Dataset Configuration
- **Dataset Name**: string (required) - Unique identifier for the dataset
- **Source Type**: enum (Synthetic, Real, Mixed) - Origin of the data
- **Modalities**: array of enums (RGB, Depth, Semantic, Instance, etc.)
- **Annotation Quality**: enum (Ground Truth, Pseudo-label, Human Label)
- **Diversity Metrics**: object
  - lighting_conditions: array of strings
  - weather_conditions: array of strings
  - object_appearances: int
  - background_complexity: int

## Relationships

- An **Isaac Sim Environment Configuration** can generate multiple **Training Dataset Configurations**
- A **Training Dataset Configuration** connects to **Isaac ROS Perception Pipeline** for model training
- A **Nav2 Humanoid Planner Configuration** integrates with **Isaac ROS Perception Pipeline** for navigation
- **Isaac ROS Perception Pipeline** processes data from **Training Dataset Configuration**

## Validation Rules

- Isaac Sim Environment name must be unique within the system
- Lighting parameters must be within physically realistic ranges
- Camera parameters must follow pinhole camera model constraints
- Locomotion constraints must be within humanoid robot capabilities
- Dataset configurations must have valid file paths for output
- Perception pipeline topics must follow ROS naming conventions