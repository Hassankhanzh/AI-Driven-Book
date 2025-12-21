---
sidebar_position: 2
title: 'ROS 2 Foundations for Physical AI'
---

# ROS 2 Foundations for Physical AI

This chapter introduces the fundamental concepts of ROS 2 architecture, which serves as the middleware connecting AI logic to robot control systems in embodied intelligence applications.

## Introduction to ROS 2

The Robot Operating System 2 (ROS 2) is not an operating system in the traditional sense, but rather a flexible framework for writing robotic software. It provides services designed specifically for robotic applications, including hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## Why ROS 2 for Physical AI?

ROS 2 is particularly well-suited for Physical AI applications because it:

- Provides a standardized communication layer between AI algorithms and robot hardware
- Offers a rich ecosystem of tools and libraries for robotics development
- Enables rapid prototyping and testing of AI-robot interactions
- Supports real-time and safety-critical applications

## Core Architecture Components

ROS 2's architecture is built around several key concepts that enable distributed robotic systems to communicate effectively:

import RosConcept from '@site/src/components/RosConcept';

<RosConcept
  title="Nodes"
  type="node"
  description="Nodes are the fundamental execution units of a ROS 2 system. Each node is a process that performs computation and communicates with other nodes through messages."
  usage="In Physical AI, nodes might represent AI decision-making algorithms, robot controllers, sensor processing units, or simulation environments."
/>

## Understanding Nodes in Depth

Nodes are the basic building blocks of any ROS 2 system. Each node handles one specific task. This approach has several benefits:

- **Separation of Concerns**: Each node handles one part of robot functionality
- **Fault Isolation**: If one node fails, the whole system does not fail
- **Scalability**: You can add new features by creating new nodes

### Node Lifecycle

ROS 2 nodes follow a clear lifecycle with four stages:

1. **Initialization**: The node starts and gets set up
2. **Activation**: The node becomes active and starts working
3. **Running**: The node does its main job
4. **Shutdown**: The node stops and cleans up

### Node Implementation Patterns

In Physical AI applications, nodes often follow these patterns:

- **Sensor Nodes**: Read from sensors and send sensor data
- **Controller Nodes**: Control robot joints or other parts
- **AI Decision Nodes**: Process sensor data and make decisions
- **Coordinator Nodes**: Manage complex behaviors using other nodes

<RosConcept
  title="Topics"
  type="topic"
  description="Topics enable asynchronous, many-to-many communication between nodes using a publish-subscribe pattern."
  usage="Particularly useful for sensor data distribution (e.g., camera feeds, LIDAR scans), control command broadcasting, and state information sharing."
/>

## Understanding Topics and Publisher-Subscriber Pattern

The publisher-subscriber pattern is the main way ROS 2 nodes communicate. In this pattern:

- **Publishers** send messages to a topic
- **Subscribers** receive messages from topics
- **ROS 2** handles delivering messages between them

This design allows flexible systems. You can add, remove, or replace nodes without affecting other parts of the system.

### Message Types and Schemas

All messages on a topic must use the same message type. These types define:

- How the data is structured
- What type each field is
- The names of each field

For example, a sensor data message might look like:

```
float64[] position
float64[] velocity
float64[] effort
```

### Quality of Service (QoS) for Topics

Topics can use different Quality of Service settings based on your needs:

- **Reliability**: Choose "best effort" (messages might be lost) or "reliable" (messages are guaranteed)
- **Durability**: Choose if new subscribers get old messages
- **History**: Choose how many messages to keep for new subscribers
- **Depth**: Choose the size of the message queue

These settings matter in Physical AI where timing and reliability needs vary.

<RosConcept
  title="Services"
  type="service"
  description="Services provide synchronous, request-response communication between nodes."
  usage="Ideal for requesting specific computations, configuration changes, or action execution with immediate feedback."
/>

## Understanding Services and Request-Response Pattern

Services let nodes communicate in a request-response way. The process works like this:

1. **Client** sends a request to a service
2. **Service** processes the request
3. **Service** sends a response back
4. **Client** gets the response and continues

This works well for tasks with a clear start and end. Use it when you need to wait for the task to finish.

### Service Definition

Services use `.srv` files to define request and response types. Each service has:

- **Request**: Data sent from the client
- **Response**: Data sent back from the service

For example, an arithmetic service:

Request:
```
int64 a
int64 b
```

Response:
```
int64 sum
```

### When to Use Services

Use services for:

- Changing settings that need confirmation
- Calculations that return results
- Tasks that must finish before continuing
- Checking system status
- Actions that give immediate feedback

### Service Characteristics

- **Synchronous**: Client waits for response
- **Reliable**: Messages are guaranteed to arrive
- **One-to-one**: One request to one service
- **Stateless**: Each request is separate

<RosConcept
  title="Actions"
  type="action"
  description="Actions are for goal-oriented, long-running tasks with feedback."
  usage="Perfect for robot navigation tasks, complex manipulation sequences, and learning episodes where progress tracking is important."
/>

## Understanding Actions for Goal-Oriented Tasks

Actions are for long tasks that need:

- **Goal requests**: To start the task
- **Feedback**: To show progress during the task
- **Result reporting**: To show the final outcome

The action process includes:

1. **Goal Request**: Client sends a goal to the action server
2. **Goal Acceptance/Rejection**: Server decides to accept the goal
3. **Execution**: Server runs the goal while giving feedback
4. **Result**: Server returns the final result

### Action Definition

Actions use `.action` files that define three message types:

- **Goal**: Data to start the action
- **Result**: Data returned when the action finishes
- **Feedback**: Data sent during execution

For example, a navigation action:

```
# Define the goal (where to go)
geometry_msgs/PoseStamped goal_pose

---
# Define the result (did it succeed?)
bool succeeded
float32 total_distance

---
# Define the feedback (current progress)
float32 distance_to_goal
geometry_msgs/Pose current_pose
```

### When to Use Actions

Use actions for:

- Navigation (moving the robot somewhere)
- Manipulation (grasping objects, opening doors)
- Calibration (finding best settings)
- Learning (training with progress tracking)
- Any long task where you want to see progress

### Action Characteristics

- **Long-running**: For tasks that take time
- **Cancellable**: Clients can stop actions early
- **Feedback-enabled**: Shows progress updates
- **Goal-oriented**: For achieving specific goals
- **Result reporting**: Returns results when done

## Communication Patterns in Depth

### Publisher-Subscriber Pattern (Topics)

The publisher-subscriber pattern is fundamental to ROS 2's design. Publishers send messages to a topic without knowing who will receive them, and subscribers receive messages from topics without knowing who published them. This decoupling allows for flexible system architectures.

### Request-Response Pattern (Services)

Services provide a synchronous request-response mechanism where a client sends a request and waits for a response. This is useful when you need to guarantee that an operation has completed before proceeding.

### Action Pattern (Actions)

Actions combine the best of both worlds - they allow for long-running operations with feedback and status updates, making them ideal for tasks that take time to complete but require monitoring.

## Middleware Role in Embodied Intelligence

ROS 2 serves as the middleware that bridges the gap between high-level AI algorithms and low-level robot control systems. This enables:

- Decoupling of AI logic from specific hardware implementations
- Reusability of AI components across different robotic platforms
- Standardized interfaces for robot perception, planning, and control
- Distributed processing capabilities for complex robotic tasks

### The Embodied Intelligence Paradigm

Embodied intelligence means intelligence comes from how an agent interacts with its environment. In robotics, this involves:

- **Perception** - connecting sensors to understanding
- **Cognition** - processing information to make decisions
- **Action** - turning decisions into movements
- **Learning** - improving behavior based on experience

ROS 2 helps by providing standard interfaces for these components to communicate.

### ROS 2 as the Integration Layer

ROS 2 works well for embodied intelligence because:

- **Hardware Abstraction**: AI code works on different robots without changes
- **Communication Standardization**: All parts use the same "language"
- **Real-time Capabilities**: Time-sensitive tasks get proper priority
- **Distributed Architecture**: Parts can run on different computers
- **Safety Mechanisms**: Built-in safety for physical systems

### Communication Patterns for Embodied Intelligence

Different patterns serve different roles:

- **Topics** for continuous sensor data and state updates
- **Services** for specific operations and settings changes
- **Actions** for complex tasks with progress tracking
- **Parameters** for system settings

This flexibility allows complex robot behaviors from simple, reusable parts.

## Practical Examples of Communication Patterns

Let's look at practical examples of how these communication patterns work in real robotic applications:

import CodeExample from '@site/src/components/CodeExample';

<CodeExample
  title="Topic Example: Sensor Data Publisher"
  description="A node that publishes sensor data to a topic for other nodes to consume."
  code={`import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(LaserScan, 'laser_scan', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = LaserScan()
        # Fill in laser scan data
        msg.ranges = [1.0, 1.5, 2.0, 2.5]  # Example ranges
        self.publisher.publish(msg)
        self.get_logger().info('Published laser scan data')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()`}
  language="python"
/>

<CodeExample
  title="Service Example: Navigation Service"
  description="A service that accepts a goal pose and returns whether navigation was successful."
  code={`import rclpy
from rclpy.node import Node
from rclpy.service import Service
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger

class NavigationService(Node):
    def __init__(self):
        super().__init__('navigation_service')
        self.srv = self.create_service(Trigger, 'start_navigation', self.navigation_callback)

    def navigation_callback(self, request, response):
        self.get_logger().info('Starting navigation task...')

        # In a real implementation, this would actually navigate the robot
        # For this example, we'll just return success
        response.success = True
        response.message = "Navigation completed successfully"

        return response

def main(args=None):
    rclpy.init(args=args)
    navigation_service = NavigationService()

    try:
        rclpy.spin(navigation_service)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_service.destroy_node()
        rclpy.shutdown()`}
  language="python"
/>

<CodeExample
  title="Action Example: Complex Manipulation Task"
  description="An action that performs a complex manipulation task with feedback."
  code={`import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory

class ManipulationAction(Node):
    def __init__(self):
        super().__init__('manipulation_action')
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'manipulation_task',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing manipulation task...')

        # Simulate the manipulation process
        feedback_msg = FollowJointTrajectory.Feedback()

        # In a real implementation, this would control the robot's manipulator
        # and provide feedback about progress
        for i in range(100):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = -1
                return result

            # Update feedback
            feedback_msg.actual.positions = [i * 0.01]  # Simulated position
            goal_handle.publish_feedback(feedback_msg)

            # Simulate work being done
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # Task completed successfully
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

def main(args=None):
    rclpy.init(args=args)
    manipulation_action = ManipulationAction()

    try:
        rclpy.spin(manipulation_action)
    except KeyboardInterrupt:
        pass
    finally:
        manipulation_action.destroy_node()
        rclpy.shutdown()`}
  language="python"
/>

## Quality of Service (QoS) Settings

ROS 2 includes Quality of Service (QoS) settings that allow you to fine-tune communication behavior based on your application's requirements:

- Reliability: Whether messages are guaranteed to be delivered
- Durability: Whether late-joining subscribers receive previous messages
- History: How many messages to store for late subscribers
- Deadline: How long a message is considered valid

These settings are crucial for Physical AI applications where timing and reliability requirements can vary significantly.

## Exercises

To reinforce your understanding of ROS 2 concepts, try the following exercises:

### Exercise 1: Node Creation
Create a simple ROS 2 node that publishes the current time to a topic called `current_time`. The message should include both the system time and a counter that increments with each publication.

### Exercise 2: Topic Communication
Design a simple sensor system with two nodes:
1. A sensor node that publishes random temperature readings to a `temperature` topic
2. A monitor node that subscribes to the `temperature` topic and prints a warning if the temperature exceeds a threshold

### Exercise 3: Service Implementation
Implement a service that accepts two numbers and returns their sum. Create both the service server and a client that calls the service with different values.

### Exercise 4: Action Implementation
Create an action that simulates a "robot cleaning" task. The action should:
- Accept a goal specifying how long the cleaning should take
- Provide feedback on the percentage of cleaning completed
- Return a result indicating whether the cleaning was successful

### Exercise 5: Quality of Service (QoS) Experiment
Experiment with different QoS settings for a publisher-subscriber pair:
- Try both "reliable" and "best effort" reliability settings
- Compare the behavior when publishing high-frequency sensor data
- Document your observations about when to use each setting

### Exercise 6: System Integration
Combine all communication patterns in a single system:
- Create a node that publishes sensor data (topic)
- Create a node that provides a service to change system parameters
- Create a node that implements an action to perform a complex task
- Create a coordinator node that uses all three communication patterns

## Summary

Understanding ROS 2 architecture is foundational for developing Physical AI systems that interact with the physical world through robotic platforms. The next chapter will demonstrate how to implement these concepts using Python.