# Data Model: 2-digital-twin-sim

**Feature**: Module 2 - The Digital Twin (Gazebo & Unity)
**Date**: 2025-12-21

## Key Entities

### Digital Twin Model
- **Name**: string (required) - Unique identifier for the digital twin
- **Description**: string - Brief description of the humanoid robot model
- **Physics Properties**: object - Configuration for physics simulation
  - gravity: float (default: -9.81)
  - collision_mesh: string (path to mesh file)
  - dynamics_coefficients: object
    - friction: float
    - damping: float
    - stiffness: float
- **ROS Integration**: object - Configuration for ROS 2 communication
  - topic_prefix: string
  - message_types: array of strings

### Physics Simulation Parameters
- **Name**: string (required) - Identifier for the physics configuration
- **Gravity**: float (default: -9.81) - Gravitational acceleration
- **Collision Detection**: object
  - enabled: boolean
  - algorithm: string (e.g., "Bullet", "ODE")
  - surface_properties: object
    - friction: float
    - bounciness: float
- **Humanoid Dynamics**: object
  - joint_constraints: array of objects
    - joint_name: string
    - limits: object
      - lower: float
      - upper: float
      - effort: float
      - velocity: float
  - center_of_mass: object (x, y, z coordinates)

### Sensor Simulation Data
- **Sensor Type**: enum (LiDAR, DepthCamera, IMU) - Type of sensor
- **Configuration**: object
  - name: string (required) - Unique sensor identifier
  - position: object (x, y, z coordinates)
  - orientation: object (quaternion: x, y, z, w)
  - parameters: object - Sensor-specific parameters
    - LiDAR: {range: float, resolution: float, fov: float}
    - DepthCamera: {width: int, height: int, fov: float, clipping_range: object}
    - IMU: {noise_variance: float, update_rate: float}
- **ROS Topic**: string - Topic name for sensor data output
- **Data Format**: string - Message type (e.g., sensor_msgs/LaserScan)

### Environment Configuration
- **Name**: string (required) - Environment identifier
- **Description**: string - Brief description of the environment
- **Objects**: array of objects
  - name: string
  - type: string (static, dynamic, interactive)
  - position: object (x, y, z coordinates)
  - mesh: string (path to mesh file)
- **Lighting**: object
  - ambient_light: float
  - directional_light: object
    - direction: object (x, y, z)
    - intensity: float
- **Physics Properties**: object
  - gravity: float
  - material_properties: array of objects

## Relationships

- A **Digital Twin Model** has one or more **Physics Simulation Parameters**
- A **Digital Twin Model** has one or more **Sensor Simulation Data** entries
- A **Digital Twin Model** exists within an **Environment Configuration**
- **Sensor Simulation Data** connects to ROS 2 topics for data flow

## Validation Rules

- Digital Twin Model name must be unique within the system
- Physics parameters must be within realistic ranges (e.g., gravity between -20 and 0)
- Sensor configurations must have valid ROS topic names (alphanumeric, underscores, forward slashes)
- Position coordinates must be valid floating-point numbers
- Sensor update rates must be positive values
- Environment objects must have valid mesh file paths