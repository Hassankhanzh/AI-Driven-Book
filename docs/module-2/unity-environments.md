---
sidebar_position: 3
title: 'Environment & Interaction in Unity'
---

# Environment & Interaction in Unity

This chapter covers creating virtual environments and human-robot interaction scenarios in Unity, including rendering and ROS 2 synchronization. Unity provides a powerful platform for creating realistic 3D environments and implementing human-robot interaction scenarios.

import SimulationConcept from '@site/src/components/SimulationConcept';
import LearningObjectives from '@site/src/components/LearningObjectives';

<LearningObjectives objectives={[
  "Create realistic 3D environments in Unity for robot simulation",
  "Implement human-robot interaction (HRI) scenarios",
  "Synchronize Unity environments with ROS 2 systems",
  "Configure Unity environment examples for simulation",
  "Troubleshoot Unity-ROS integration issues"
]} />

## Introduction to Unity for Robotics

Unity is a versatile game engine that has found significant applications in robotics simulation and digital twin development. Its powerful rendering capabilities, physics engine, and extensive asset library make it ideal for creating realistic virtual environments for humanoid robots.

<SimulationConcept
  title="Unity in Robotics"
  type="unity"
  description="Unity provides realistic rendering, physics simulation, and interaction capabilities for robotics applications."
  usage="Essential for creating visually accurate digital twins with proper rendering and HRI scenarios."
/>

## Environment Rendering in Unity

Unity's rendering pipeline allows for the creation of highly realistic environments that can accurately simulate real-world conditions for robot perception and navigation.

### Setting Up a Basic Environment

To create a basic environment in Unity for robotics simulation:

1. **Create a new 3D project** in Unity Hub
2. **Import necessary assets** for your environment (terrain, objects, lighting)
3. **Configure the physics settings** to match real-world conditions
4. **Set up lighting** to simulate real-world illumination

### Terrain and Environment Assets

Unity supports various types of environmental elements:

- **Terrain**: For creating landscapes with realistic textures
- **ProBuilder**: For creating custom architectural elements
- **3D Models**: Importing custom objects via FBX, OBJ, or other formats
- **Prefabs**: Reusable environment components

### Lighting Configuration

Proper lighting is crucial for realistic rendering and accurate sensor simulation:

```csharp
// Example: Setting up directional light to simulate sun
public class EnvironmentLighting : MonoBehaviour
{
    public Light sunLight;
    public float intensity = 1.0f;
    public Color lightColor = Color.white;

    void Start()
    {
        if (sunLight == null)
            sunLight = GetComponent<Light>();

        sunLight.intensity = intensity;
        sunLight.color = lightColor;
        sunLight.type = LightType.Directional;
    }
}
```

### Post-Processing Effects

For enhanced realism, Unity's post-processing stack can be used:

- **Ambient Occlusion**: Improves depth perception
- **Bloom**: Simulates light scattering
- **Color Grading**: Adjusts color tone to match real-world conditions

## Human-Robot Interaction (HRI) Scenarios

Human-robot interaction in Unity involves creating scenarios where humans and robots can interact in the virtual environment, which is crucial for testing HRI algorithms.

### Interaction Mechanisms

Unity provides several ways to implement HRI:

1. **Raycasting**: Detecting objects and interactions in the environment
2. **Trigger Colliders**: Detecting when objects enter specific areas
3. **UI Elements**: Providing interfaces for human-robot communication
4. **Animation Systems**: Simulating robot behaviors and responses

### Example HRI Implementation

```csharp
// Example: Simple HRI system for robot response to human presence
using UnityEngine;

public class HumanRobotInteraction : MonoBehaviour
{
    public GameObject robot;
    public float interactionDistance = 2.0f;
    public LayerMask humanLayer;

    private void Update()
    {
        // Detect nearby humans using sphere cast
        Collider[] nearbyHumans = Physics.OverlapSphere(
            transform.position,
            interactionDistance,
            humanLayer
        );

        if (nearbyHumans.Length > 0)
        {
            // Robot responds to human presence
            OnHumanDetected(nearbyHumans[0].transform);
        }
    }

    private void OnHumanDetected(Transform human)
    {
        // Implement robot response logic
        Vector3 direction = (human.position - robot.transform.position).normalized;
        robot.transform.LookAt(human);

        // Send ROS message about human detection
        // (Integration with ROS would go here)
    }
}
```

### Behavior Trees for HRI

For complex interaction scenarios, behavior trees can be implemented:

- **Sequence Nodes**: Execute actions in order
- **Selector Nodes**: Choose between different action options
- **Decorator Nodes**: Modify behavior based on conditions
- **Service Nodes**: Run continuous checks

## ROS 2 Synchronization with Unity

Connecting Unity to ROS 2 enables real-time data exchange between the simulation and ROS 2 systems.

### ROS# (ROS Sharp) Integration

ROS# is a popular Unity package for ROS 2 integration. It provides:

- **Message Publishing/Subscribing**: Send and receive ROS messages
- **Service Calls**: Execute ROS services from Unity
- **Parameter Management**: Access ROS parameters
- **TF Transformations**: Synchronize coordinate frames

### Setting Up ROS# Connection

```csharp
// Example: Basic ROS# publisher setup
using RosSharp.RosBridgeClient;
using RosSharp.Messages.Geometry;

public class UnityROSPublisher : MonoBehaviour
{
    private RosSocket rosSocket;
    private string robotPositionTopic = "/unity_robot_position";

    void Start()
    {
        // Initialize ROS connection
        WebSocketSimpleClient webSocket = new WebSocketSimpleClient(WebSocketProtocol.ws, "localhost", 9090);
        rosSocket = new RosSocket(webSocket);

        // Create publisher
        rosSocket.Advertise<Pose>(robotPositionTopic);
    }

    void Update()
    {
        // Publish robot position
        Pose robotPose = new Pose();
        robotPose.position.x = transform.position.x;
        robotPose.position.y = transform.position.y;
        robotPose.position.z = transform.position.z;

        rosSocket.Publish(robotPositionTopic, robotPose);
    }
}
```

### Synchronization Considerations

When synchronizing Unity with ROS 2:

- **Frame Rate**: Balance visual quality with ROS message frequency
- **Coordinate Systems**: Ensure Unity's coordinate system aligns with ROS conventions
- **Timing**: Handle time synchronization between Unity and ROS
- **Data Types**: Map Unity data types to ROS message types

## Unity Environment Configuration Examples

### Indoor Environment Setup

```csharp
// Example: Configuring an indoor environment
using UnityEngine;

public class IndoorEnvironment : MonoBehaviour
{
    public GameObject[] furniturePrefabs;
    public Transform environmentParent;

    [System.Serializable]
    public class RoomConfiguration
    {
        public string roomName;
        public Vector3 roomSize;
        public Vector3 roomPosition;
        public Color ambientLightColor;
    }

    public RoomConfiguration[] roomConfigs;

    void Start()
    {
        foreach (RoomConfiguration config in roomConfigs)
        {
            CreateRoom(config);
        }
    }

    void CreateRoom(RoomConfiguration config)
    {
        // Create room boundaries
        GameObject room = new GameObject(config.roomName);
        room.transform.position = config.roomPosition;

        // Add lighting
        Light mainLight = room.AddComponent<Light>();
        mainLight.type = LightType.Directional;
        mainLight.color = config.ambientLightColor;
        mainLight.intensity = 1.0f;
    }
}
```

### Outdoor Environment Setup

For outdoor environments, consider:

- **Skybox configuration**: Use realistic sky materials
- **Weather systems**: Implement dynamic weather conditions
- **Terrain generation**: Use Unity's terrain tools for landscapes
- **Vegetation**: Add realistic plant models

## Exercises

### Exercise 1: Environment Creation
Create a simple indoor environment in Unity with walls, furniture, and proper lighting. Export the environment as a prefab that can be instantiated in different scenes.

### Exercise 2: HRI Implementation
Implement a basic HRI scenario where a robot responds to a human entering its personal space. The robot should turn to face the human and send a ROS message indicating the detection.

### Exercise 3: ROS Integration
Set up a ROS# connection in Unity and implement a simple publisher that sends the position of a moving object to a ROS topic.

## Troubleshooting Unity-ROS Integration

### Common Connection Issues

1. **WebSocket Connection Failures**:
   - Verify ROS bridge server is running: `ros2 run rosbridge_server rosbridge_websocket`
   - Check firewall settings
   - Ensure correct IP address and port configuration

2. **Coordinate System Mismatches**:
   - Unity uses left-handed coordinate system (Y-up)
   - ROS uses right-handed coordinate system (Z-up)
   - Implement proper coordinate transformation

3. **Performance Issues**:
   - Reduce rendering quality during simulation
   - Limit the frequency of ROS message publishing
   - Use object pooling for frequently instantiated objects

### Debugging Tips

- Use Unity's console for error messages
- Monitor ROS topics with `ros2 topic echo`
- Implement logging in Unity scripts to track message flow
- Use Unity's Profiler to identify performance bottlenecks

## Summary

Unity provides powerful capabilities for creating realistic environments and implementing human-robot interaction scenarios. By properly configuring rendering, implementing HRI systems, and synchronizing with ROS 2, you can create sophisticated digital twin environments for humanoid robots. The combination of Unity's visual capabilities and ROS 2's robotics framework enables comprehensive simulation and testing of robot systems.